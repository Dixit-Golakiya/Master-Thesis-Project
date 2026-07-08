/**
 ******************************************************************************
 * @file    capacitance_analysis.h
 ******************************************************************************
 */
#ifndef CAPACITANCE_ANALYSIS_H
#define CAPACITANCE_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* ── Size limits ─────────────────────────────────────────────────────────── */
#define SMALL_SAMPLE_COUNT	10
#define MAX_WINDOWS			1
#define MAX_SAMPLES      	2200/SMALL_SAMPLE_COUNT
#define MAX_WIN_SAMPLES  	MAX_SAMPLES/MAX_WINDOWS
#define NUM_CAPACITORS     	8
#define ANALYSIS_RESULT_BUF_SIZE    2200u * MAX_WINDOWS

/* ── Window detection ────────────────────────────────────────────────────── */
#define SPIKE_THRESHOLD_MA          30
#define CURRENT_LEVEL_THRESHOLD_MA 100
#define NOISE_FLOOR_MA              50
#define MIN_WINDOW_DURATION_MS     500
#define MIN_VOLTAGE_DELTA_MV         5

/* Sliding window search config */
#define SLIDE_STEP_MS        50      /* 10ms step  */
#define SLIDE_WIN_LENGTH_MS  200     /* 200ms window  */
#define SLIDE_MIN_OFFSET_MS  0       /* 20ms min start offset */
#define SLIDE_R2_MIN         0.99f
#define SLIDE_RMSE_MAX_MV    1.0f    /* 1mV */

/* ── Capacitance regression window ──────────────────────────────────────────
 *    t >= t_win_start + 0.5 s   →  t_reg_start_ms = t_win_start_ms + 500
 *    t <= t_win_end   * 0.9     →  t_reg_end_ms   = t_win_end_ms   * 0.9
 * ─────────────────────────────────────────────────────────────────────────*/
#define CAP_REG_START_OFFSET_MS  500     /* added to window start (ms) */
#define CAP_REG_END_FACTOR       0.9f   /* fraction of absolute window-end */

/* ── ESR window ──────────────────────────────────────────────────────────── */
#define ESR_START_OFFSET_MS  300
#define ESR_END_OFFSET_MS   1500
#define MIN_CURRENT_MA        10
#define PRE_DISCHARGE_N        10   /* samples averaged before window for V_pre */

/* ── Data structures ─────────────────────────────────────────────────────── */

typedef struct {
	int32_t timestamp_us;
    int16_t voltage[NUM_CAPACITORS];  /* mV */
    float	current_i1;               /* mA */
    float  	current_i2;               /* mA */
    float  	current_i3;               /* mA */
} DataSample_t;

typedef struct {
    float    slope;
    float    intercept;
    float    r_squared;
    float    rmse;
    uint16_t n;
    bool     valid;
} RegressionResult_t;

/**
 * Per-capacitor results for one discharge window.
 *
 * C_eq  = mean(|I2|) / |slope|              — cumulative series value
 * C_ind = series_capacitance_diff(C_eq[])   — individual capacitor value
 */
typedef struct {
    uint8_t  channel;

    /* Capacitance */
    float    C_eq;     /* F — cumulative */
    float    C_ind;    /* F — individual */

    /* Regression diagnostics */
    float    slope;    /* V/s  */
    float	 slope_ind;
    float    r2_C;
    float    rmse_c_mv;  /* mV   */
    float    I2_avg;   /* A    */

    /* ESR */
    float    V_pre;    /* V — avg before window      */
    float    V_pred;   /* V — intercept at t_rel=0   */
    float    ESR_I1;   /* mΩ — individual            */
    float    ESR_I2;   /* mΩ — individual            */
    float    ESR_I3;   /* mΩ — individual            */
    float    I1_avg;   /* A */
    float    I3_avg;   /* A */
    float    r2_esr;
    float    rmse_esr_mv;
    float    t_base_esr;
    float    t_predict_abs;
} CapacitorResult_t;

typedef struct {
    uint16_t	start_idx;
    uint16_t	end_idx;
    float		t_start_ms;
    float		t_end_ms;
    float		duration_ms;
    char     	type[16];
    bool     	valid;
} DischargeWindow_t;

typedef struct {
    DischargeWindow_t win;
    CapacitorResult_t cap[NUM_CAPACITORS];
    bool              complete;
} WindowResult_t;

typedef struct {
    DataSample_t      samples[MAX_SAMPLES];
    uint16_t          sample_count;
    bool              threshold_reached;
    uint16_t          threshold_low_mv;
    bool              analysis_complete;
    DischargeWindow_t detected_windows[MAX_WINDOWS];
    uint8_t           window_count;
    WindowResult_t    results[MAX_WINDOWS];
    uint8_t           results_count;
    float _t[MAX_WIN_SAMPLES];
    float _v[MAX_WIN_SAMPLES];
    float _I[MAX_SAMPLES];
    float _dI[MAX_SAMPLES];
} AnalysisEngine_t;


/** Reset and init. threshold_low_mv = 0 → no threshold, fill to MAX_SAMPLES. */
void     Analysis_Init                (AnalysisEngine_t *e, uint16_t threshold_low_mv);

/** Add one sample. Returns false when collection should stop. */
bool     Analysis_AddSample           (AnalysisEngine_t *e, uint32_t ts_ms,
                                       const int16_t v[NUM_CAPACITORS],
                                       float I1, float I2, float I3);

/** Detect all discharge/charge windows. Returns count. */
uint8_t  Analysis_DetectAllWindows    (AnalysisEngine_t *e);

/** Regression on all detected windows. Returns true if ≥1 succeeded. */
bool     Analysis_PerformAllRegressions(AnalysisEngine_t *e);

/** Format results to text buffer for UART. Returns bytes written. */
uint16_t Analysis_FormatResults       (AnalysisEngine_t *e, char *buf, uint16_t bufsz);

/** Reset for next run (preserves threshold setting). */
void     Analysis_Reset               (AnalysisEngine_t *e);

#endif /* CAPACITANCE_ANALYSIS_H */
