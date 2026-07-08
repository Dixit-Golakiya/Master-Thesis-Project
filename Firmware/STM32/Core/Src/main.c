/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with double buffering and raw data
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "capacitance_analysis.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC1_CHANNELS 7
#define ADC2_CHANNELS 4
#define ADC5_CHANNELS 3

#define DEFAULT_SAMPLE_COUNT 500
#define SMALL_SAMPLE_DURATION_MS 40000

#define CALIBRATION_DURATION_MS 5000
#define CALIBRATION_BASE_VALUE 2047

//~0.5A: 1003 | ~1A: 2047 | ~1.5A: 3130 | ~1.95A: 4095
#define DAC_VALUE	4095

// Voltage thresholds for GPIO control
#define THRESHOLD_HIGH	2600  // ~2.6V
#define THRESHOLD_LOW	THRESHOLD_HIGH*0.866   // ~2.2516V, 75% Energy

#define DEBOUNCE_DELAY_MS	500

// Offset value for current
#define IC1_OFFSET -9.7
#define IC2_OFFSET -3.8
#define IC3_OFFSET -3.0

// VREFINT calibration value
#define VREFINT_CAL (*((uint16_t*)VREFINT_CAL_ADDR))

#define LOAD_RESISTANCE_OHMS   12.0f

/* ── S4 Load Type: change this line to switch mode ──
 * Options: LOAD_TYPE_RESISTIVE
 *          LOAD_TYPE_CC
 * ─────────────────────────────────────────────────── */
static AnalysisEngine_t g_analysis;
volatile bool analysis_trigger_pending = false;
static char analysis_result_buf[ANALYSIS_RESULT_BUF_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Double buffers for ADC
uint16_t adc1_buffer[2][ADC1_CHANNELS];
uint16_t adc2_buffer[2][ADC2_CHANNELS];
uint16_t adc5_buffer[2][ADC5_CHANNELS];

// Accumulation buffers for averaging
uint32_t adc1_accumulator[ADC1_CHANNELS];
uint32_t adc2_accumulator[ADC2_CHANNELS];
uint32_t adc5_accumulator[ADC5_CHANNELS];

// Averaged results
uint16_t adc1_averaged[ADC1_CHANNELS];
uint16_t adc2_averaged[ADC2_CHANNELS];
uint16_t adc5_averaged[ADC5_CHANNELS];

uint32_t start_time = 0;

volatile uint16_t sample_count = 0;

volatile uint8_t adc_half_flags = 0;
volatile uint8_t adc_full_flags = 0;

volatile bool measurement_running = false;

volatile uint32_t last_button_press = 0;
volatile bool toggle_sample_count_pending = false;

volatile bool use_small_sample = false;  // Flag to track PF1 state
volatile bool small_sample_active = false;  // Flag to track if temporary mode is active
volatile uint32_t small_sample_start_time = 0;  // When small sample mode was activated

static uint64_t avg_start_timestamp = 0;

volatile bool mode_transition_pending = false;
volatile uint32_t tim2_overflow_count = 0;

volatile bool calibration_in_progress = false;
volatile uint32_t calibration_start_time = 0;
volatile float ic1_cal_sum = 0;
volatile float ic2_cal_sum = 0;
volatile float ic3_cal_sum = 0;
volatile float calibration_sample_count = 0;
volatile float ic1_offset_cal = IC1_OFFSET;
volatile float ic2_offset_cal = IC2_OFFSET;
volatile float ic3_offset_cal = IC3_OFFSET;

volatile bool voltage_safe = false;

typedef struct {
    uint32_t timestamp;
    uint8_t adc1_idx;
    uint8_t adc2_idx;
    uint8_t adc5_idx;
} DataSnapshot_t;

volatile DataSnapshot_t pending_data;
volatile bool data_ready_to_send = false;
volatile bool pending_transmission = false;

volatile bool dac_enabled = false;
volatile uint8_t pulse_state = 0;

static uint8_t  rt_windows_completed = 0;
static bool     rt_window_active     = false;

typedef enum {
    LOAD_TYPE_RESISTIVE = 0,
    LOAD_TYPE_CC        = 1
} LoadType_t;

volatile bool      resistive_analysis_active = false;
volatile LoadType_t s4_load_type             = LOAD_TYPE_RESISTIVE; /* toggles on each S4 arm */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void TransmitDataPacket(void);
static void UpdateGPIOOutputs(uint16_t V10, uint16_t V20, uint16_t V30,
                              uint16_t V40, uint16_t V50, uint16_t V60,
                              uint16_t V70, uint16_t V08, uint16_t V09);
static void CheckSmallSampleTimeout(void);
static void ActivateSmallSampleMode(void);
static void DeactivateSmallSampleMode(void);
static void ResetMeasurement(void);
static void StartMeasurement(void);
static void StopMeasurement(void);
static void StartCalibration(void);
static void CheckAndCompleteCalibration(void);
static void EmergencyStop(void);
static void FeedAnalysisSample(uint16_t V10, uint16_t V20, uint16_t V30,
                               uint16_t V40, uint16_t V50, uint16_t V60,
                               uint16_t V70, uint16_t V08,
                               int16_t  IC1, int16_t  IC2, int16_t  IC3,
                               uint32_t timestamp_us);
uint64_t GetTimerTicks(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CheckAndCompleteCalibration(void)
{
    if (!calibration_in_progress)
        return;

    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - calibration_start_time;

    if (elapsed_time >= CALIBRATION_DURATION_MS) {
        calibration_in_progress = false;

        // Turn off calibration LED
        HAL_GPIO_WritePin(LD4_GPIO_Port, GPIO_PIN_4, GPIO_PIN_RESET);

        // Stop ADC and timer
        HAL_TIM_Base_Stop(&htim3);
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_ADC_Stop_DMA(&hadc5);

        if (calibration_sample_count > 0) {
            // Calculate average raw values
            float ic1_avg = ic1_cal_sum / calibration_sample_count;
            float ic2_avg = ic2_cal_sum / calibration_sample_count;
            float ic3_avg = ic3_cal_sum / calibration_sample_count;

            ic1_offset_cal = (float)ic1_avg - CALIBRATION_BASE_VALUE;
            ic2_offset_cal = (float)ic2_avg - CALIBRATION_BASE_VALUE;
            ic3_offset_cal = (float)ic3_avg - CALIBRATION_BASE_VALUE;

            char msg[256];
            snprintf(msg, sizeof(msg),
                     "Calibration complete!\r\nIC1 offset: %.1f, IC2 offset: %.1f, IC3 offset: %.1f\r\n",
                     ic1_offset_cal, ic2_offset_cal, ic3_offset_cal);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

        } else {
            const char* msg = "Calibration failed: no samples collected\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        // Reset accumulators
        ic1_cal_sum = 0;
        ic2_cal_sum = 0;
        ic3_cal_sum = 0;
        calibration_sample_count = 0;
    }
}


static void StartCalibration(void)
{
    if (measurement_running) {
        const char* msg = "Cannot calibrate during measurement. Stop measurement first.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    if (calibration_in_progress)
        return;

    const char* msg = "Calibration started (5 seconds)...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    calibration_in_progress = true;
    calibration_start_time = HAL_GetTick();
    calibration_sample_count = 0;
    ic1_cal_sum = 0;
    ic2_cal_sum = 0;
    ic3_cal_sum = 0;

    // Start ADC DMA (measurement is not running at this point)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS*2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC2_CHANNELS*2);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_buffer, ADC5_CHANNELS*2);
    HAL_TIM_Base_Start_IT(&htim3);
}


static void UpdateGPIOOutputs(uint16_t V10, uint16_t V20, uint16_t V30,
                              uint16_t V40, uint16_t V50, uint16_t V60,
                              uint16_t V70, uint16_t V08, uint16_t V09)
{
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

    if (((V10-V20)<=THRESHOLD_LOW) || ((V20-V30)<=THRESHOLD_LOW) || ((V30-V40)<=THRESHOLD_LOW) ||
        ((V40-V50)<=THRESHOLD_LOW) || ((V50-V60)<=THRESHOLD_LOW) || ((V60-V70)<=THRESHOLD_LOW) ||
        ((V70-V08)<=THRESHOLD_LOW) || (V08<=THRESHOLD_LOW) )
    {
        if (voltage_safe)
        {
            voltage_safe = false;

            const char* msg = "Voltage dropped below LOW threshold: DAC locked.\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        if (dac_enabled) {
            dac_enabled = false;
            DeactivateSmallSampleMode();
            HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
            HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);

            analysis_trigger_pending = true;
        }
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

        return;
    }

    if (!voltage_safe) {
        voltage_safe = true;
        const char* msg = "Voltage recovered above LOW threshold: DAC unlocked.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

    if (((V10-V20)>=THRESHOLD_HIGH) || ((V20-V30)>=THRESHOLD_HIGH) || ((V30-V40)>=THRESHOLD_HIGH) ||
        ((V40-V50)>=THRESHOLD_HIGH) || ((V50-V60)>=THRESHOLD_HIGH) || ((V60-V70)>=THRESHOLD_HIGH) ||
        ((V70-V08)>=THRESHOLD_HIGH) || (V08>=THRESHOLD_HIGH))
    {
    	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    }


    if (!dac_enabled) {
    	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    }
}

static void AccumulateSamples(void)
{
    // Get current buffer indices
    uint8_t idx1 = pending_data.adc1_idx;
    uint8_t idx2 = pending_data.adc2_idx;
    uint8_t idx5 = pending_data.adc5_idx;

    // Accumulate ADC1 channels
    for (int i = 0; i < ADC1_CHANNELS; i++) {
        adc1_accumulator[i] += adc1_buffer[idx1][i];
    }
    // Accumulate ADC2 channels
    for (int i = 0; i < ADC2_CHANNELS; i++) {
        adc2_accumulator[i] += adc2_buffer[idx2][i];
    }
    // Accumulate ADC5 channels
    for (int i = 0; i < ADC5_CHANNELS; i++) {
        adc5_accumulator[i] += adc5_buffer[idx5][i];
    }
    if (sample_count == 0) {
        avg_start_timestamp = pending_data.timestamp;
    }
    sample_count++;
}

static void CalculateAverages(uint16_t num_samples)
{
    // Calculate averages for ADC1
    for (int i = 0; i < ADC1_CHANNELS; i++) {
        adc1_averaged[i] = adc1_accumulator[i] / num_samples;
        adc1_accumulator[i] = 0;  // Reset accumulator
    }
    // Calculate averages for ADC2
    for (int i = 0; i < ADC2_CHANNELS; i++) {
        adc2_averaged[i] = adc2_accumulator[i] / num_samples;
        adc2_accumulator[i] = 0;  // Reset accumulator
    }
    // Calculate averages for ADC5
    for (int i = 0; i < ADC5_CHANNELS; i++) {
        adc5_averaged[i] = adc5_accumulator[i] / num_samples;
        adc5_accumulator[i] = 0;  // Reset accumulator
    }
    sample_count = 0;  // Reset sample counter
}


static void CheckSmallSampleTimeout(void)
{
    if (small_sample_active && measurement_running) {
        uint32_t current_time = HAL_GetTick();
        if ((current_time - small_sample_start_time) >= SMALL_SAMPLE_DURATION_MS) {
            DeactivateSmallSampleMode();
        }
    }
}

static void ActivateSmallSampleMode(void)
{
    if (!measurement_running) {
        return;  // Can only activate during measurement
    }
    // Always reset for fresh sampling
    memset(adc1_accumulator, 0, sizeof(adc1_accumulator));
    memset(adc2_accumulator, 0, sizeof(adc2_accumulator));
    memset(adc5_accumulator, 0, sizeof(adc5_accumulator));
    sample_count = 0;
    mode_transition_pending = false;
    small_sample_active = true;
    use_small_sample = true;
    small_sample_start_time = HAL_GetTick();

}

static void DeactivateSmallSampleMode(void)
{
    small_sample_active = false;
    use_small_sample = false;
    mode_transition_pending = false;
}

static void ResetMeasurement(void)
{
    // Reset accumulators
    memset(adc1_accumulator, 0, sizeof(adc1_accumulator));
    memset(adc2_accumulator, 0, sizeof(adc2_accumulator));
    memset(adc5_accumulator, 0, sizeof(adc5_accumulator));
    sample_count = 0;
    avg_start_timestamp = 0;
    DeactivateSmallSampleMode();
    // Reset GPIO LEDs
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    tim2_overflow_count = 0;
}

static void StartMeasurement(void)
{
    if (measurement_running)
        return;
    measurement_running = true;
    ResetMeasurement();
    dac_enabled = false;
    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    float actual_time_ms = ((htim3.Init.Prescaler + 1.0f) * (htim3.Init.Period + 1.0f)) / 16000.0f;
    char msg[128];
    snprintf(msg, sizeof(msg), "Measurement started (averaging %u samples), Sample Time: %.3f ms\r\n",
                 DEFAULT_SAMPLE_COUNT, actual_time_ms);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS*2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC2_CHANNELS*2);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_buffer, ADC5_CHANNELS*2);
}

static void StopMeasurement(void)
{
    measurement_running = false;
    data_ready_to_send = false;
    pending_transmission = false;
    analysis_trigger_pending = false;
    dac_enabled = false;
    pulse_state = 0;

    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);

    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    HAL_TIM_Base_Stop_IT(&htim3);

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc5);

    ResetMeasurement();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    const char* msg = "Measurement stopped\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static void EmergencyStop(void)
{
    measurement_running = false;
    dac_enabled = false;

    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    HAL_TIM_Base_Stop(&htim3);

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc5);

    ResetMeasurement();

    const char* msg = "Emergency stopped\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static void TransmitDataPacket(void)
{
    if (!data_ready_to_send && !pending_transmission)
        return;

    uint64_t ticks = GetTimerTicks();
    double elapsed_us = (double)ticks / 16.0f;
    uint64_t time_int = (uint64_t)elapsed_us;
    // Clear the data ready flag NOW
    data_ready_to_send = false;
    pending_transmission = false;
    // Check if small sample mode should timeout
    CheckSmallSampleTimeout();

    if (use_small_sample) {
        // ===== SMALL SAMPLE MODE =====
        AccumulateSamples();

        if (sample_count >= SMALL_SAMPLE_COUNT) {
            CalculateAverages(SMALL_SAMPLE_COUNT);

            uint16_t R1  = adc1_averaged[6];
            uint16_t R5  = adc5_averaged[2];

            float Vref1 = 3.0f * VREFINT_CAL * 1000.0f / R1;
            float Vref5 = 3.0f * VREFINT_CAL * 1000.0f / R5;

            float IC1_f = (float)adc1_averaged[2];
            float IC2_f = (float)adc1_averaged[1];
            float IC3_f = (float)adc2_averaged[1];

            IC1_f = (((IC1_f - ic1_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (0.2f);
            IC2_f = -(((IC2_f - ic2_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (18.34f * 0.038f);
            IC3_f = (((IC3_f - ic3_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (18.22f * 0.0372f);
            float V09_f = IC2_f * 0.038f;

            float fV10 = (adc1_averaged[4] * Vref1 * 8.0f / 4095.0f) - V09_f;
            float fV20 = (adc2_averaged[2] * Vref1 * 7.0f / 4095.0f) - V09_f;
            float fV30 = (adc1_averaged[3] * Vref1 * 6.0f / 4095.0f) - V09_f;
            float fV40 = (adc2_averaged[3] * Vref5 * 5.0f / 4095.0f) - V09_f;
            float fV50 = (adc2_averaged[0] * Vref5 * 4.0f / 4095.0f) - V09_f;
            float fV60 = (adc5_averaged[0] * Vref1 * 3.0f / 4095.0f) - V09_f;
            float fV70 = (adc5_averaged[1] * Vref1 * 2.0f / 4095.0f) - V09_f;
            float fV08 = (adc1_averaged[0] * Vref1 * 1.0f / 4095.0f) - V09_f;

            uint16_t V10 = (fV10 > 0.0f) ? (uint16_t)fV10 : 0;
            uint16_t V20 = (fV20 > 0.0f) ? (uint16_t)fV20 : 0;
            uint16_t V30 = (fV30 > 0.0f) ? (uint16_t)fV30 : 0;
            uint16_t V40 = (fV40 > 0.0f) ? (uint16_t)fV40 : 0;
            uint16_t V50 = (fV50 > 0.0f) ? (uint16_t)fV50 : 0;
            uint16_t V60 = (fV60 > 0.0f) ? (uint16_t)fV60 : 0;
            uint16_t V70 = (fV70 > 0.0f) ? (uint16_t)fV70 : 0;
            uint16_t V08 = (fV08 > 0.0f) ? (uint16_t)fV08 : 0;

            int16_t IC1 = (int16_t)IC1_f;
            int16_t IC2 = (int16_t)IC2_f;
            int16_t IC3 = (int16_t)IC3_f;
            int16_t V09 = (int16_t)V09_f;

            UpdateGPIOOutputs(V10,V20,V30,V40,V50,V60,V70,V08,V09);

            if (dac_enabled && !analysis_trigger_pending) {
                FeedAnalysisSample(V10, V20, V30, V40, V50, V60, V70, V08,
                                   IC1, IC2, IC3,
                                   (uint32_t)time_int);
            }
        }
    } else {
        // ===== DEFAULT AVERAGING MODE =====
        AccumulateSamples();

        if (sample_count >= DEFAULT_SAMPLE_COUNT) {
            CalculateAverages(DEFAULT_SAMPLE_COUNT);

            uint64_t t_end = pending_data.timestamp;

            // midpoint = t1 + (t2 - t1)/2
            pending_data.timestamp =
                avg_start_timestamp + (t_end - avg_start_timestamp) / 2;

            // Handle mode transition: small sample -> averaging
            if (mode_transition_pending) {
                mode_transition_pending = false;
                use_small_sample = true;

                for(int i=0;i<ADC1_CHANNELS;i++) adc1_accumulator[i]=0;
                for(int i=0;i<ADC2_CHANNELS;i++) adc2_accumulator[i]=0;
                for(int i=0;i<ADC5_CHANNELS;i++) adc5_accumulator[i]=0;
                sample_count = 0;
                avg_start_timestamp = 0;

                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                data_ready_to_send = true;
                return;
            }

            uint16_t R1  = adc1_averaged[6];
            uint16_t R5  = adc5_averaged[2];

            float Vref1 = 3.0f * VREFINT_CAL * 1000.0f / R1;
            float Vref5 = 3.0f * VREFINT_CAL * 1000.0f / R5;

            float IC1_f = (float)adc1_averaged[2];
            float IC2_f = (float)adc1_averaged[1];
            float IC3_f = (float)adc2_averaged[1];

            IC1_f = (((IC1_f - ic1_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (0.2f);
            IC2_f = -(((IC2_f - ic2_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (18.34f * 0.038f);
            IC3_f = (((IC3_f - ic3_offset_cal) * Vref1 / 4095.0f) - (Vref1 / 2.0f)) / (18.22f * 0.0372f);
            float V09_f = IC2_f * 0.038f;

            float fV10 = (adc1_averaged[4] * Vref1 * 8.0f / 4095.0f) - V09_f;
            float fV20 = (adc2_averaged[2] * Vref1 * 7.0f / 4095.0f) - V09_f;
            float fV30 = (adc1_averaged[3] * Vref1 * 6.0f / 4095.0f) - V09_f;
            float fV40 = (adc2_averaged[3] * Vref5 * 5.0f / 4095.0f) - V09_f;
            float fV50 = (adc2_averaged[0] * Vref5 * 4.0f / 4095.0f) - V09_f;
            float fV60 = (adc5_averaged[0] * Vref1 * 3.0f / 4095.0f) - V09_f;
            float fV70 = (adc5_averaged[1] * Vref1 * 2.0f / 4095.0f) - V09_f;
            float fV08 = (adc1_averaged[0] * Vref1 * 1.0f / 4095.0f) - V09_f;

            uint16_t V10 = (fV10 > 0.0f) ? (uint16_t)fV10 : 0;
            uint16_t V20 = (fV20 > 0.0f) ? (uint16_t)fV20 : 0;
            uint16_t V30 = (fV30 > 0.0f) ? (uint16_t)fV30 : 0;
            uint16_t V40 = (fV40 > 0.0f) ? (uint16_t)fV40 : 0;
            uint16_t V50 = (fV50 > 0.0f) ? (uint16_t)fV50 : 0;
            uint16_t V60 = (fV60 > 0.0f) ? (uint16_t)fV60 : 0;
            uint16_t V70 = (fV70 > 0.0f) ? (uint16_t)fV70 : 0;
            uint16_t V08 = (fV08 > 0.0f) ? (uint16_t)fV08 : 0;

            int16_t IC1 = (int16_t)IC1_f;
            int16_t IC2 = (int16_t)IC2_f;
            int16_t IC3 = (int16_t)IC3_f;
            int16_t V09 = (int16_t)V09_f;

            UpdateGPIOOutputs(V10,V20,V30,V40,V50,V60,V70,V08,V09);

            if (dac_enabled && !analysis_trigger_pending) {
                FeedAnalysisSample(V10, V20, V30, V40, V50, V60, V70, V08,
                                   IC1, IC2, IC3,
                                   (uint32_t)time_int);
            }
        }
    }
}

uint64_t GetTimerTicks(void)
{
    uint32_t low;
    uint32_t high;

    __disable_irq();
    high = tim2_overflow_count;
    low  = __HAL_TIM_GET_COUNTER(&htim2);
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        high++;
        low = __HAL_TIM_GET_COUNTER(&htim2);
    }
    __enable_irq();

    return ((uint64_t)high << 32) | low;
}


static void FeedAnalysisSample(uint16_t V10, uint16_t V20, uint16_t V30,
                               uint16_t V40, uint16_t V50, uint16_t V60,
                               uint16_t V70, uint16_t V08,
                               int16_t  IC1, int16_t  IC2, int16_t  IC3,
                               uint32_t timestamp_us)
{
    /* ── Real-time window detection ────────────────────────────────────────
     * Count completed discharge windows BEFORE feeding to analysis engine.
     * ─────────────────────────────────────────────────────────────────── */
    float I_now = (float)(IC2 < 0 ? -IC2 : IC2) +
                  (float)(IC3 < 0 ? -IC3 : IC3);   /* |IC2| + |IC3| in mA */

    if (!rt_window_active) {
        /* Window OPENS when current exceeds level threshold */
        if (I_now > (float)CURRENT_LEVEL_THRESHOLD_MA) {
            rt_window_active = true;
        }
    } else {
        /* Window CLOSES when current drops below noise floor */
        if (I_now < (float)NOISE_FLOOR_MA) {
            rt_window_active = false;
            rt_windows_completed++;

            /* ── MAX_WINDOWS reached: stop collecting, trigger analysis ── */
            if (rt_windows_completed >= MAX_WINDOWS) {
                analysis_trigger_pending = true;
                return;   
            }
        }
    }

    /* ── Already triggered, don't feed any more samples ── */
    if (analysis_trigger_pending)
        return;

    /* ── Feed sample to analysis engine ── */
    int16_t voltages[NUM_CAPACITORS] = {
        (int16_t)V10, (int16_t)V20, (int16_t)V30, (int16_t)V40,
        (int16_t)V50, (int16_t)V60, (int16_t)V70, (int16_t)V08
    };

    bool keep_going = Analysis_AddSample(&g_analysis,
                                          timestamp_us,
                                          voltages,
                                          IC1, IC2, IC3);
    if (!keep_going) {
        analysis_trigger_pending = true;
    }
}

static void RunAnalysisAndReport(void)
{
    analysis_trigger_pending = false;
    bool was_resistive = resistive_analysis_active;
    /* ── Stop ONLY DAC/PWM, keep measurement alive ── */
    dac_enabled               = false;
    pulse_state               = 0;
    resistive_analysis_active = false;
    rt_windows_completed      = 0;
    rt_window_active          = false;

    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    /* ── Reset only sample accumulators, NOT TIM2 ── */
    memset(adc1_accumulator, 0, sizeof(adc1_accumulator));
    memset(adc2_accumulator, 0, sizeof(adc2_accumulator));
    memset(adc5_accumulator, 0, sizeof(adc5_accumulator));
    sample_count        = 0;
    avg_start_timestamp = 0;
    DeactivateSmallSampleMode();

    /* ── Raw CSV ── */
    const char *hdr =
        "\r\n=== RAW DISCHARGE DATA ===\r\n"
        "Time, V10, V20, V30, V40, V50, V60, V70, V08, V09, IC1, IC2, IC3\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)hdr, strlen(hdr), HAL_MAX_DELAY);
    char line[128];
    for (uint16_t i = 0; i < g_analysis.sample_count; i++) {
        DataSample_t *s = &g_analysis.samples[i];
        int n = snprintf(line, sizeof(line),
            "%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%.0f,%.0f,%.0f\r\n",
            s->timestamp_us/1000.0f,
            s->voltage[0], s->voltage[1], s->voltage[2], s->voltage[3],
            s->voltage[4], s->voltage[5], s->voltage[6], s->voltage[7],
			s->current_i2*0.038f,
            s->current_i1, s->current_i2, s->current_i3);
        if (n > 0)
            HAL_UART_Transmit(&huart2, (uint8_t*)line, n, HAL_MAX_DELAY);
    }
    /* ── Sanity check ── */
    if (g_analysis.sample_count < 50) {
        const char *msg = "ERROR: Too few samples.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    /* ── Detect all windows ── */
    uint8_t nwin = Analysis_DetectAllWindows(&g_analysis);
    {
        char tmp[64];
        int n = snprintf(tmp, sizeof(tmp),
                         "\r\nDetected %u window(s), %u samples collected.\r\n",
                         (unsigned)nwin, (unsigned)g_analysis.sample_count);
        if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)tmp, (uint16_t)n, HAL_MAX_DELAY);
    }

    if (nwin == 0) {
        /* Fallback: treat entire dataset as one discharge window */
        const char *msg = "WARNING: No window found. Using full dataset.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        DischargeWindow_t *dw = &g_analysis.detected_windows[0];
        dw->start_idx   = 0;
        dw->end_idx     = (uint16_t)(g_analysis.sample_count - 1);
        dw->t_start_ms  = g_analysis.samples[0].timestamp_us/1000.0f;
        dw->t_end_ms    = g_analysis.samples[g_analysis.sample_count-1].timestamp_us/1000.0f;
        dw->duration_ms = dw->t_end_ms - dw->t_start_ms;
        dw->valid       = true;
        strncpy(dw->type, "discharge", 15);
        dw->type[15]    = '\0';
        g_analysis.window_count = 1;
    }

    /* ── Regression on all windows ── */
    if (!Analysis_PerformAllRegressions(&g_analysis)) {
        const char *msg = "ERROR: Regression failed.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    if (was_resistive  && g_analysis.window_count > 0)
    {
        resistive_analysis_active = false;
        const char* sep = "\r\n=== CAPACITANCE & ESR RESULTS ===\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)sep, strlen(sep), HAL_MAX_DELAY);
        char mode_line[64];
        snprintf(mode_line, sizeof(mode_line), "Mode: %s load  |  R = %.2f Ohm\r\n",
                (s4_load_type == LOAD_TYPE_RESISTIVE) ? "RESISTIVE" : "CONST-CURRENT",
                LOAD_RESISTANCE_OHMS);
        HAL_UART_Transmit(&huart2, (uint8_t*)mode_line, strlen(mode_line), HAL_MAX_DELAY);

        for (uint8_t w = 0; w < g_analysis.window_count; w++)
        {
            DischargeWindow_t *dw = &g_analysis.detected_windows[w];
                if (!dw->valid) continue;

                /* ── Guard: need at least 2 samples in window ── */
                if (dw->end_idx <= dw->start_idx) continue;

                DataSample_t *s0 = &g_analysis.samples[dw->start_idx];      /* first sample  */
                DataSample_t *s1 = &g_analysis.samples[dw->start_idx + 1];  /* second sample */
                DataSample_t *sE = &g_analysis.samples[dw->end_idx];        /* last sample   */

                /* ── Pack voltage = V10 (index 0, highest node) ── */
                float V0_mV   = (float)s0->voltage[0];   /* mV at window start  */
                float V_end_mV= (float)sE->voltage[0];   /* mV at window end    */

                /* ── Initial current from sensors (mA) ── */
                float I0_mA = fabsf((float)s0->current_i2)
                            + fabsf((float)s0->current_i3);
                float I0_A  = I0_mA * 1e-3f;

                /* ── Window timing ── */
                float t_span_s = dw->duration_ms * 1e-3f;

                /* ── ESR: instantaneous voltage drop at load connection ──────────
                 * V_oc = last sample BEFORE window, V0 = first sample under load
                 * ESR = ΔV / I₀
                 * ──────────────────────────────────────────────────────────────── */
                float V_oc_mV = (dw->start_idx > 0)
                              ? (float)g_analysis.samples[dw->start_idx + 2].voltage[0]
                              : V0_mV;
                float dV_drop_mV = V_oc_mV - V0_mV;
                float ESR_ohm    = (I0_A > 0.001f)
                                 ? ((dV_drop_mV) / I0_A)
                                 : ((-dV_drop_mV )/I0_A);

                /* ── Capacitance ─────────────────────────────────────────────────
                 * RESISTIVE:  V(t) = V0·e^(-t/RC)  →  τ = -t/ln(Vend/V0)
                 *             C = τ / R
                 *
                 * CONST-CURR: V(t) = V0 - (I/C)·t  (linear)
                 *             C = I·Δt / ΔV
                 * ──────────────────────────────────────────────────────────────── */
                float C_F = -1.0f;

                if (s4_load_type == LOAD_TYPE_RESISTIVE)
                {
                    char dbg[128];
                    snprintf(dbg, sizeof(dbg),
                        "  [DBG] V0=%.0f V_end=%.0f t_span=%.4f s\r\n",
                        V0_mV, V_end_mV, t_span_s);
                    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
                    /* Use two-point τ as sanity check */
                    float V1_mV = (float)s1->voltage[0];
                    float dt_s  = (s1->timestamp_us - s0->timestamp_us) * 1e-6f;
                    float tau_step = -1.0f;
                    tau_step = -dt_s / logf(V1_mV / V0_mV);

                    /* Full-window τ is more robust */
                    float tau_full = -1.0f;
                    tau_full = -t_span_s / logf(V_end_mV / V0_mV);
                    C_F = (tau_full / LOAD_RESISTANCE_OHMS);

                    char res[256];
                    snprintf(res, sizeof(res),
                        "Win %u [RESISTIVE]: V_oc=%.0f mV, V_start=%.0f mV, V_end=%.0f mV, t=%.1f ms\r\n"
                        "  ESR = %.1f Ohm  (dV=%.1f mV, I0=%.1f mA)\r\n"
                        "  tau = %.1f s  |  C = %.2f F\r\n"
                        "  tau_step = %.5f s  (2-point sanity)\r\n",
                        (unsigned)w,
                        V_oc_mV, V0_mV, V_end_mV, dw->duration_ms,
                        ESR_ohm, dV_drop_mV, I0_mA,
                        (tau_full),
                        (C_F),
                        (tau_step));
                    HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), HAL_MAX_DELAY);
                }
                else  /* LOAD_TYPE_CC */
                {
                    /* Average current over window for better accuracy */
                    float I_avg_mA = 0.0f;
                    uint16_t count = 0;
                    for (uint16_t i = dw->start_idx; i <= dw->end_idx; i++) {
                        I_avg_mA += fabsf((float)g_analysis.samples[i].current_i2)
                                  + fabsf((float)g_analysis.samples[i].current_i3);
                        count++;
                    }
                    if (count > 0) I_avg_mA /= (float)count;
                    float I_avg_A = I_avg_mA * 1e-3f;

                    /* C = I·Δt / ΔV  (linear discharge model) */
                    float dV_mV = V0_mV - V_end_mV;   /* total voltage drop over window */
                    if (dV_mV > 0.0f && I_avg_A > 0.001f)
                        C_F = (I_avg_A * t_span_s) / (dV_mV * 1e-3f);

                    /* dV/dt linearity check: compare first-half vs second-half slope */
                    uint16_t mid_idx = (dw->start_idx + dw->end_idx) / 2;
                    float V_mid_mV   = (float)g_analysis.samples[mid_idx].voltage[0];
                    float t_half_s   = t_span_s / 2.0f;
                    float slope1     = (t_half_s > 0.0f) ? ((V0_mV - V_mid_mV) / t_half_s) : 0.0f;
                    float slope2     = (t_half_s > 0.0f) ? ((V_mid_mV - V_end_mV) / t_half_s) : 0.0f;
                    float linearity  = (slope1 > 0.0f) ? (slope2 / slope1) : 0.0f;
                    /* linearity ≈ 1.0 → good CC, far from 1.0 → not true CC load */

                    char res[320];
                    snprintf(res, sizeof(res),
                        "Win %u [CONST-CURR]: V_oc=%.0f mV, V_start=%.0f mV, V_end=%.0f mV, t=%.1f ms\r\n"
                        "  ESR   = %.4f Ohm  (dV=%.1f mV, I0=%.1f mA)\r\n"
                        "  I_avg = %.2f mA  |  dV = %.1f mV  |  C = %.2f F\r\n"
                        "  Linearity check (slope2/slope1) = %.3f  (1.0 = ideal CC)\r\n",
                        (unsigned)w,
                        V_oc_mV, V0_mV, V_end_mV, dw->duration_ms,
                        (ESR_ohm > 0.0f ? ESR_ohm : 0.0f), dV_drop_mV, I0_mA,
                        I_avg_mA, dV_mV,
                        (C_F > 0.0f ? C_F : 0.0f),
                        linearity);
                    HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), HAL_MAX_DELAY);
                }

                /* ── Per-cell ESR (common to both modes) ── */
                if (I0_A > 0.001f)
                {
                    const char* cell_hdr = "  Per-cell ESR:\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t*)cell_hdr, strlen(cell_hdr), HAL_MAX_DELAY);
                    char cell_line[128];
                    uint8_t prev_idx = (dw->start_idx > 0) ? (dw->start_idx - 1) : dw->start_idx;
                    float tau = -1.0f;

                    for (uint8_t c = 0; c < 8; c++) {
                        float Voc_cell  = (float)(g_analysis.samples[prev_idx].voltage[c])*1e-3;
                        float Vld_cell  = (float)(s0->voltage[c])*1e-3;
                        float esr_cell  = (Voc_cell - Vld_cell) / I0_A;
                        float V_start 	= (float)(g_analysis.samples[dw->start_idx].voltage[c])*1e-3;
                        float V_end 	= (float)(g_analysis.samples[dw->end_idx].voltage[c])*1e-3;

                        /* Full-window τ is more robust */
                        tau = -t_span_s / logf(V_end / V_start);
                        C_F = (tau / LOAD_RESISTANCE_OHMS);

                        snprintf(cell_line, sizeof(cell_line),
                            "    V%u: ESR = %3.1f mOhm, (%3.3f, %3.3f), %3.3f, %3.3f, %3.2f, %4.4f\r\n",
                            (unsigned)(c+1),   /* V10, V20 ... V70 */
                            (esr_cell > 0.0f ? esr_cell : -esr_cell),
							Vld_cell,Voc_cell,
							V_start,
							V_end, C_F, tau);
                        HAL_UART_Transmit(&huart2, (uint8_t*)cell_line, strlen(cell_line), HAL_MAX_DELAY);
                    }

                }
            }
    }
    /* ── Print results ── */
    uint16_t len = Analysis_FormatResults(&g_analysis,
                                           analysis_result_buf,
                                           ANALYSIS_RESULT_BUF_SIZE);
    if (len > 0)
        HAL_UART_Transmit(&huart2, (uint8_t*)analysis_result_buf, len, HAL_MAX_DELAY);

    /* ── Reset for next run ── */
    Analysis_Reset(&g_analysis);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ADC12_COMMON->CCR |= ADC_CCR_VREFEN;
  ADC345_COMMON->CCR |= ADC_CCR_VREFEN;
  HAL_Delay(10);
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
         HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
         HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED) != HAL_OK) {
         Error_Handler();
     }
     HAL_TIM_Base_Start_IT(&htim3);
     HAL_TIM_Base_Start_IT(&htim2);

     HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
     HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

     // Start ADC DMA conversions
     HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS*2);
     HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC2_CHANNELS*2);
     HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_buffer, ADC5_CHANNELS*2);

     HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

     const char* header = "Time, V10, V20, V30, V40, V50, V60, V70, V08, V09, IC1, IC2, IC3\r\n";
     HAL_UART_Transmit(&huart2, (uint8_t*)header, strlen(header), HAL_MAX_DELAY);
     HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (analysis_trigger_pending) {
	              RunAnalysisAndReport();
	          }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (calibration_in_progress && !measurement_running) {
        ic1_cal_sum += adc1_buffer[0][2];  // IC1 is at index 2 in ADC1
        ic2_cal_sum += adc1_buffer[0][1];  // IC2 is at index 1 in ADC1
        ic3_cal_sum += adc2_buffer[0][1];  // IC3 is at index 1 in ADC2
        calibration_sample_count++;
        CheckAndCompleteCalibration();
        return;
    }

    if (!measurement_running)
        return;

    uint64_t current_ticks = GetTimerTicks();

    // Half-transfer means buffer[0] is ready
    if(hadc->Instance == ADC1) {
        adc_half_flags |= 0x01;
    } else if(hadc->Instance == ADC2) {
        adc_half_flags |= 0x02;
    } else if(hadc->Instance == ADC5) {
        adc_half_flags |= 0x08;
    }

    if(adc_half_flags == 0x0B) {
        // All ADCs half-transfer complete
        pending_data.adc1_idx = 0;
        pending_data.adc2_idx = 0;
        pending_data.adc5_idx = 0;
        pending_data.timestamp = current_ticks;
        adc_half_flags = 0;  // Reset for next cycle
        data_ready_to_send = true;
        TransmitDataPacket();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (calibration_in_progress && !measurement_running) {
        ic1_cal_sum += adc1_buffer[1][2];  // IC1 is at index 2 in ADC1
        ic2_cal_sum += adc1_buffer[1][1];  // IC2 is at index 1 in ADC1
        ic3_cal_sum += adc2_buffer[1][1];  // IC3 is at index 1 in ADC2
        calibration_sample_count++;
        CheckAndCompleteCalibration();
        return;
    }

    if (!measurement_running)
        return;
    uint64_t current_ticks = GetTimerTicks();

    // Full-transfer means buffer[1] is ready
    if(hadc->Instance == ADC1) {
        adc_full_flags |= 0x01;
    } else if(hadc->Instance == ADC2) {
        adc_full_flags |= 0x02;
    } else if(hadc->Instance == ADC5) {
        adc_full_flags |= 0x08;
    }

    if(adc_full_flags == 0x0B) {
        // All ADCs full-transfer complete
        pending_data.adc1_idx = 1;
        pending_data.adc2_idx = 1;
        pending_data.adc5_idx = 1;
        pending_data.timestamp = current_ticks;
        adc_full_flags = 0;  // Reset for next cycle
        data_ready_to_send = true;
        TransmitDataPacket();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        tim2_overflow_count++;         // TIM2 overflow occurred - increment counter
    }
    else if(htim->Instance == TIM3)
    {

    }

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
     if(htim->Instance == TIM4)
     {
    	 if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    	 {
    		 if (pulse_state == 0) {
    			 ActivateSmallSampleMode();
    		 } else {
    			 DeactivateSmallSampleMode();
    		 }
    	 }
    	 else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    	 {
    		 if (pulse_state == 0)
    		 {
    			 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_VALUE);
    			 pulse_state = 1;
    		 }
    		 else
    		 {
    			 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    			 pulse_state = 0;
    		 }
    	 }
     }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_button_press) < DEBOUNCE_DELAY_MS)
        return;
    last_button_press = current_time;

    if (GPIO_Pin == S1_Pin)
    {
    	EmergencyStop();
    }

    else if (GPIO_Pin == S2_Pin)
    {
        if (calibration_in_progress)
            return;

        if (!measurement_running) {
            StartMeasurement();
        } else {
            StopMeasurement();
        }
    }

    else if (GPIO_Pin == S3_Pin)
    {
    	StartCalibration();
    }
    else if (GPIO_Pin == S4_Pin)
    {
        if (!measurement_running) {
            const char* msg = "Cannot start analysis. Start measurement first (S2).\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        if (dac_enabled) {
            dac_enabled               = false;
            resistive_analysis_active = false;
            rt_windows_completed      = 0;
            rt_window_active          = false;
            Analysis_Reset(&g_analysis);
            DeactivateSmallSampleMode();
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
            char cancel_msg[64];
            snprintf(cancel_msg, sizeof(cancel_msg), "S4: %s discharge cancelled.\r\n",
                (s4_load_type == LOAD_TYPE_RESISTIVE) ? "CR" : "CC");
            HAL_UART_Transmit(&huart2, (uint8_t*)cancel_msg, strlen(cancel_msg), HAL_MAX_DELAY);
            return;
        }
        if (!voltage_safe) {
            const char* msg = "S4 blocked: voltage not in safe region yet.\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        Analysis_Init(&g_analysis, THRESHOLD_LOW);
        rt_windows_completed      = 0;
        rt_window_active          = false;
        resistive_analysis_active = true;
        dac_enabled               = true;

        pulse_state = 0;

        memset(adc1_accumulator, 0, sizeof(adc1_accumulator));
        memset(adc2_accumulator, 0, sizeof(adc2_accumulator));
        memset(adc5_accumulator, 0, sizeof(adc5_accumulator));
        sample_count        = 0;
        avg_start_timestamp = 0;

        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        ActivateSmallSampleMode();   

        char arm_msg[96];
        snprintf(arm_msg, sizeof(arm_msg),
            "S4 armed: %s load mode.\r\nSwitch load now. Capturing discharge...\r\n",
            (s4_load_type == LOAD_TYPE_RESISTIVE) ? "RESISTIVE" : "CONST-CURRENT");
        HAL_UART_Transmit(&huart2, (uint8_t*)arm_msg, strlen(arm_msg), HAL_MAX_DELAY);
    }
    else if (GPIO_Pin == S5_Pin)  // PC1: S5
    {
        if (!measurement_running) {
            const char* msg = "Start measurement first (S2).\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        if (dac_enabled) {
            dac_enabled               = false;
            rt_windows_completed      = 0;
            rt_window_active          = false;
            pulse_state               = 0;
            analysis_trigger_pending  = false;

            HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4);
            HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

            Analysis_Reset(&g_analysis);
            DeactivateSmallSampleMode();
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

            const char* msg = "S5: CC discharge cancelled.\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
        if (!voltage_safe) {
            const char* msg = "S5 blocked: voltage not in safe region yet.\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        Analysis_Init(&g_analysis, THRESHOLD_LOW);
        rt_windows_completed = 0;
        rt_window_active     = false;
        dac_enabled          = true;

        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);

        memset(adc1_accumulator, 0, sizeof(adc1_accumulator));
        memset(adc2_accumulator, 0, sizeof(adc2_accumulator));
        memset(adc5_accumulator, 0, sizeof(adc5_accumulator));
        sample_count        = 0;
        avg_start_timestamp = 0;

        HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

        pulse_state = 0;
        HAL_TIM_PWM_Init(&htim4);
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);

        const char *msg = "Discharge started. Collecting data...\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) {
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
