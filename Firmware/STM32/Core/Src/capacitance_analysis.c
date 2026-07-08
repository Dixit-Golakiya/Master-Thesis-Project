#include "capacitance_analysis.h"
#include <stdio.h>

static inline float _fabs(float x)  { return x < 0.0f ? -x : x; }
static inline int   _isnan(float x) { return x != x; }

/* ============================================================================
 * LINEAR REGRESSION  y = slope·x + intercept
 * ============================================================================ */
static void LinReg(const float *x, const float *y, uint16_t n,
                   RegressionResult_t *r)
{
    r->valid = false;
    if (n < 3) return;

    float sx=0,sy=0,sxy=0,sx2=0,sy2=0;
    for (uint16_t i=0;i<n;i++){
        float xi=x[i], yi=y[i];
        sx+=xi; sy+=yi; sxy+=xi*yi; sx2+=xi*xi; sy2+=yi*yi;
    }
    float N = n;
    float den = N*sx2 - sx*sx;
    if (fabsf(den) < 1e-9f) return;

    float sl = (N*sxy - sx*sy) / den;
    float ic = (sy - sl*sx) / N;

    float ss_tot = sy2 - sy*sy/N;
    float ss_res = 0.0f;
    for (uint16_t i=0;i<n;i++){
        float e = y[i] - (ic + sl*x[i]);
        ss_res += e*e;
    }

    r->slope     = sl;
    r->intercept = ic;
    r->r_squared = (fabsf(ss_tot)<1e-9f) ? 0.f : (1.0f - ss_res/ss_tot);
    r->rmse      = sqrtf(ss_res/N);
    r->n         = n;
    r->valid     = true;
}

/* ============================================================================
 * PRE-DISCHARGE VOLTAGE
 * ============================================================================ */
static float PreV(const AnalysisEngine_t *e, float ws, uint8_t cap)
{
    uint16_t n = (ws >= PRE_DISCHARGE_N) ? PRE_DISCHARGE_N : ws;
    if (n == 0) return e->samples[0].voltage[cap] / 1000.0f;
    float s = 0.0f;
    for (uint16_t i = ws-n; i < ws; i++)
        s += e->samples[i].voltage[cap];
    return (s / n) / 1000.0f;
}

/* ============================================================================
 * CAPACITANCE SLIDING WINDOW SEARCH
 * Slides a sub-window of length win_len_ms across the search range
 * [t_search_start_ms, t_search_end_ms] in steps of SLIDE_STEP_MS.
 *
 * Each position runs one linregress and scores by:
 *   Tier 0 (perfect)  : R² >= SLIDE_R2_MIN  AND  RMSE <= SLIDE_RMSE_MAX_MV
 *   Tier 1 (r2 only)  : R² >= SLIDE_R2_MIN  AND  RMSE >  SLIDE_RMSE_MAX_MV
 *   Tier 2 (rmse only): R² <  SLIDE_R2_MIN  AND  RMSE <= SLIDE_RMSE_MAX_MV
 *   Tier 3 (fallback) : best R² seen (neither threshold met)
 *
 * If NO sliding position fits (win_len >= search_range), falls back to
 *
 * Parameters (all in ms):
 *   t_search_start_ms = w->t_start_ms + 500     (mask lower bound)
 *   t_search_end_ms   = w->t_end_ms   * 0.9     (mask upper bound, ABSOLUTE)
 *   win_len_ms        = search_range  * 0.2     (sub-window, fits inside range)
 * ============================================================================ */
static void SlidingCapSearch(AnalysisEngine_t *e, uint8_t cap,
                              float t_search_start_ms,
                              float t_search_end_ms,
                              float win_len_ms,
                              CapacitorResult_t *res)
{
    /* ── Tier 0: perfect (R² AND RMSE both meet threshold) ── */
    float best_r2      = -1.0f, best_rmse  = 1e9f;
    float best_C       =  0.0f, best_slope = 0.0f, best_I  = 0.0f;
    bool  found_perfect = false;

    /* ── Tier 1: R² only ── */
    float r2o_r2       = -1.0f, r2o_rmse   = 1e9f;
    float r2o_C        =  0.0f, r2o_slope  = 0.0f, r2o_I  = 0.0f;
    bool  found_r2only  = false;

    /* ── Tier 2: RMSE only ── */
    float rmo_rmse     =  1e9f, rmo_r2     = -1.0f;
    float rmo_C        =  0.0f, rmo_slope  = 0.0f, rmo_I  = 0.0f;
    bool  found_rmseonly = false;

    /* ── Tier 3: pure fallback (best R² regardless) ── */
    float fb_r2        = -1.0f, fb_rmse    =  1e9f;
    float fb_C         =  0.0f, fb_slope   = 0.0f, fb_I   = 0.0f;

    bool  any_iteration = false;
    float search_range  = t_search_end_ms - t_search_start_ms;

    /* ── If win_len >= search_range: single regression over full range ── */
    if (win_len_ms >= search_range) {
        win_len_ms = search_range;
    }

    for (float ts = t_search_start_ms;
         ts + win_len_ms <= t_search_end_ms + 0.5f;  /* +0.5 for float tolerance */
         ts += (float)SLIDE_STEP_MS)
    {
        float te = ts + win_len_ms;
        if (te > t_search_end_ms) te = t_search_end_ms;

        uint16_t cnt   = 0;
        float    I_sum = 0.0f;
        double   t_base = -1.0;

        for (uint16_t k = 0; k < e->sample_count && cnt < MAX_WIN_SAMPLES; k++) {
            float t_ms = (float)e->samples[k].timestamp_us/1000.0f;
            if (t_ms < ts)  continue;
            if (t_ms > te)  break;

            if (t_base < 0.0) t_base = (double)t_ms / 1000.0f;

            e->_t[cnt] = (double)t_ms / 1000.0f - t_base;
            e->_v[cnt] = (double)e->samples[k].voltage[cap] / 1000.0f;

            float i2 = _fabs((float)e->samples[k].current_i2);
            float i3 = _fabs((float)e->samples[k].current_i3);
            I_sum += (i2 > i3) ? i2 : i3;
            cnt++;
        }

        if (cnt < 3 || t_base < 0.0) continue;

        float I_avg = (I_sum / cnt) / 1000.0f;   /* mA → A */
        if (I_avg < 1e-6f) continue;

        RegressionResult_t reg;
        LinReg(e->_t, e->_v, cnt, &reg);
        if (!reg.valid || _fabs(reg.slope) < 1e-9f) continue;

        any_iteration = true;
        float rmse_mv = reg.rmse * 1000.0f;
        float C       = I_avg / _fabs(reg.slope);
        bool  good_r2   = (reg.r_squared >= SLIDE_R2_MIN);
        bool  good_rmse = (rmse_mv       <= SLIDE_RMSE_MAX_MV);

        /* ── Tier 0: both thresholds met — keep best R², ties by RMSE ── */
        if (good_r2 && good_rmse) {
            if (!found_perfect ||
                reg.r_squared > best_r2 ||
                (reg.r_squared == best_r2 && rmse_mv < best_rmse))
            {
                found_perfect = true;
                best_r2 = reg.r_squared; best_rmse = rmse_mv;
                best_C  = C; best_slope = reg.slope; best_I = I_avg;
            }
        }
        /* ── Tier 1: R² only — keep best R² ── */
        else if (good_r2) {
            if (!found_r2only || reg.r_squared > r2o_r2) {
                found_r2only = true;
                r2o_r2 = reg.r_squared; r2o_rmse = rmse_mv;
                r2o_C  = C; r2o_slope = reg.slope; r2o_I = I_avg;
            }
        }
        /* ── Tier 2: RMSE only — keep lowest RMSE ── */
        else if (good_rmse) {
            if (!found_rmseonly || rmse_mv < rmo_rmse) {
                found_rmseonly = true;
                rmo_rmse = rmse_mv; rmo_r2 = reg.r_squared;
                rmo_C    = C; rmo_slope = reg.slope; rmo_I = I_avg;
            }
        }

        /* ── Always track best R² for tier 3 fallback ── */
        if (reg.r_squared > fb_r2) {
            fb_r2 = reg.r_squared; fb_rmse = rmse_mv;
            fb_C  = C; fb_slope = reg.slope; fb_I = I_avg;
        }
    }

    if (!any_iteration) return;   /* no valid window at all */

    /* ── Select result by tier priority ── */
    float sel_r2, sel_rmse, sel_C, sel_slope, sel_I;

    if (found_perfect) {
        sel_r2=best_r2; sel_rmse=best_rmse;
        sel_C=best_C;   sel_slope=best_slope; sel_I=best_I;
    } else if (found_r2only) {
        sel_r2=r2o_r2;  sel_rmse=r2o_rmse;
        sel_C=r2o_C;    sel_slope=r2o_slope;  sel_I=r2o_I;
    } else if (found_rmseonly) {
        sel_r2=rmo_r2;  sel_rmse=rmo_rmse;
        sel_C=rmo_C;    sel_slope=rmo_slope;  sel_I=rmo_I;
    } else {
        sel_r2=fb_r2;   sel_rmse=fb_rmse;
        sel_C=fb_C;     sel_slope=fb_slope;   sel_I=fb_I;
    }

    res->slope     = sel_slope;
    res->r2_C      = sel_r2;
    res->rmse_c_mv = sel_rmse;
    res->I2_avg    = sel_I;
    res->C_eq      = sel_C;
}

/* ============================================================================
 * SERIES_CAPACITANCE_DIFF
 * ============================================================================ */
static void SeriesDiff(const float *C_eq, float *C_ind, uint8_t len)
{
    double prev_inv = 0.0;
    for (int k = (int)len - 1; k >= 0; k--) {
        double ceq = C_eq[k];
        if (ceq <= 0.0f || _isnan((float)ceq)) { C_ind[k] = 0.0f; continue; }
        double inv  = 1.0f / ceq;
        double diff = inv - prev_inv;
        if (diff > 1e-9) {
            C_ind[k] = (float)(1.0f / diff);
            prev_inv = inv;
        } else {
            C_ind[k] = 0.0f;
        }
    }
}

/* ============================================================================
 * ESR REGRESSION
 * ============================================================================ */
static void CalcESR(AnalysisEngine_t *e, uint8_t cap,
                    const DischargeWindow_t *w,
                    CapacitorResult_t *res)
{
    float t0 = w->t_start_ms + ESR_START_OFFSET_MS;
    float t1 = w->t_start_ms + ESR_END_OFFSET_MS;
    if (t1 > w->t_end_ms) t1 = w->t_end_ms;

    uint16_t cnt = 0;
    float    s1=0, s2=0, s3=0;
    double   t_base = -1.0;
    bool     first  = true;

    for (uint16_t k = 0; k < e->sample_count && cnt < MAX_WIN_SAMPLES; k++) {
        float ts = (float)e->samples[k].timestamp_us/1000.0f;
        if (ts < t0 || ts > t1) continue;
        if (first) { t_base = (double)ts / 1000.0f; first = false; }
        e->_t[cnt] = (double)ts / 1000.0f - t_base;
        e->_v[cnt] = (double)e->samples[k].voltage[cap] / 1000.0f;
        s1 += _fabs((float)e->samples[k].current_i1);
        s2 += _fabs((float)e->samples[k].current_i2);
        s3 += _fabs((float)e->samples[k].current_i3);
        cnt++;
    }
    if (cnt < 3 || first) return;

    RegressionResult_t reg;
    LinReg(e->_t, e->_v, cnt, &reg);
    if (!reg.valid) return;

    res->r2_esr      = reg.r_squared;
    res->rmse_esr_mv = reg.rmse * 1000.0f;
    res->t_base_esr  = (float)t_base;

    float  t_predict_ms;
    if (w->start_idx > 0)
        t_predict_ms = e->samples[w->start_idx - 1].timestamp_us/1000.0f;
    else
        t_predict_ms = w->t_start_ms;

    res->t_predict_abs = (float)t_predict_ms / 1000.0f;

    uint32_t t_base_ms  = (uint32_t)(t_base * 1000.0f + 0.5f);
    float t_predict_rel = (float)((int32_t)t_predict_ms - (int32_t)t_base_ms) / 1000.0f;
    res->V_pred = reg.intercept + reg.slope * t_predict_rel;

    float Vdrop = res->V_pre - res->V_pred;
    if (Vdrop <= 0.0005f) return;

    float I1 = (s1/cnt)/1000.0f, I2 = (s2/cnt)/1000.0f, I3 = (s3/cnt)/1000.0f;
    float Im = (float)MIN_CURRENT_MA / 1000.0f;
    res->I1_avg = I1;
    res->I3_avg = I3;
    res->ESR_I1 = (I1>Im) ? (Vdrop/I1)*1000.0f : 0.0f;
    res->ESR_I2 = (I2>Im) ? (Vdrop/I2)*1000.0f : 0.0f;
    res->ESR_I3 = (I3>Im) ? (Vdrop/I3)*1000.0f : 0.0f;
}

/* ============================================================================
 * WINDOW DETECTION
 * ============================================================================ */
static uint8_t DetectWindows(AnalysisEngine_t *e)
{
    uint16_t n = e->sample_count;
    if (n < 10) return 0;
    if (n > MAX_SAMPLES) n = MAX_SAMPLES;

    for (uint16_t i=0;i<n;i++)
        e->_I[i] = (_fabs((float)e->samples[i].current_i2) +
                    _fabs((float)e->samples[i].current_i3)) / 1000.0f;

    e->_dI[0] = 0.0f;
    for (uint16_t i=1;i<n;i++){
        float dt = (float)(e->samples[i].timestamp_us -
                           e->samples[i-1].timestamp_us) / 1000.0f;
        e->_dI[i] = (dt > 0.0f) ? (e->_I[i]-e->_I[i-1])/dt : 0.0f;
    }

    const float spike = (float)SPIKE_THRESHOLD_MA/1000.0f;
    const float level = (float)CURRENT_LEVEL_THRESHOLD_MA/1000.0f;
    const float noise = (float)NOISE_FLOOR_MA/1000.0f;

    uint8_t wc=0; bool active=false; uint16_t ws=0;

    for (uint16_t i=1; i<n; i++){
        bool eod = (i == n-1);
        if (!active){
            if (_fabs(e->_dI[i]) > spike || e->_I[i] > level)
                { ws=i; active=true; }
        } else {
            if (e->_I[i] < noise || eod){
                uint16_t we;
                if (eod) we = i;
                else     we = (i > ws + 2) ? (i - 2) : ws;

                if (e->samples[we].timestamp_us/1000.0f <= e->samples[ws].timestamp_us/1000.0f){
                    active = false; continue;
                }

                uint32_t dur_ms = e->samples[we].timestamp_us/1000.0f
                                - e->samples[ws].timestamp_us/1000.0f;
                float vd = _fabs((float)(e->samples[we].voltage[0] -
                                         e->samples[ws].voltage[0]));

                if (dur_ms >= (uint32_t)MIN_WINDOW_DURATION_MS &&
                    vd  >= (float)MIN_VOLTAGE_DELTA_MV &&
                    wc  <  MAX_WINDOWS)
                {
                    DischargeWindow_t *dw = &e->detected_windows[wc];
                    dw->start_idx   = ws;
                    dw->end_idx     = we;
                    dw->t_start_ms  = e->samples[ws].timestamp_us/1000.0f;
                    dw->t_end_ms    = e->samples[we].timestamp_us/1000.0f;
                    dw->duration_ms = (float)dur_ms;
                    dw->valid       = true;
                    int16_t vc = e->samples[we].voltage[0] - e->samples[ws].voltage[0];
                    strncpy(dw->type, vc<0?"discharge":"charge", 15);
                    dw->type[15] = '\0';
                    wc++;
                }
                active = false;
            }
        }
    }
    e->window_count = wc;
    return wc;
}

/* ============================================================================
 * ANALYSE ONE WINDOW
 * ============================================================================ */
static bool AnalyseWindow(AnalysisEngine_t *e,
                           const DischargeWindow_t *w,
                           WindowResult_t *wr)
{
    wr->win      = *w;
    wr->complete = false;

    /* ── Search range boundaries  ──
     *   mask_full = (time >= t_start+0.5) & (time <= t_end_abs*0.9)
     *
     *   t_search_start = t_start_ms + 500       (t_start + 0.5s)
     *   t_search_end   = t_end_ms   * 0.9       (ABSOLUTE t_end * 0.9)
     */
    float t_search_start = (float)w->t_start_ms + (float)CAP_REG_START_OFFSET_MS;
    float t_search_end   = (float)w->t_end_ms   * CAP_REG_END_FACTOR;   /* ABSOLUTE *0.9 */
    float win_len_ms   = t_search_end - t_search_start;

    if (t_search_end <= t_search_start) {
        t_search_end = (float)w->t_end_ms;
        win_len_ms = t_search_end - t_search_start;
    }

    float C_eq_arr [NUM_CAPACITORS];
    float ESR1_arr [NUM_CAPACITORS];
    float ESR2_arr [NUM_CAPACITORS];
    float ESR3_arr [NUM_CAPACITORS];

    for (uint8_t cap = 0; cap < NUM_CAPACITORS; cap++) {
        CapacitorResult_t *res = &wr->cap[cap];
        memset(res, 0, sizeof(CapacitorResult_t));
        res->channel = cap;
        res->V_pre   = PreV(e, w->start_idx, cap);

        SlidingCapSearch(e, cap, t_search_start, t_search_end, win_len_ms, res);
        C_eq_arr[cap] = res->C_eq;

        CalcESR(e, cap, w, res);
        ESR1_arr[cap] = res->ESR_I1;
        ESR2_arr[cap] = res->ESR_I2;
        ESR3_arr[cap] = res->ESR_I3;
    }

    /* ── SeriesDiff: C_eq[] → C_ind[] ── */
    float C_ind_arr[NUM_CAPACITORS];
    SeriesDiff(C_eq_arr, C_ind_arr, NUM_CAPACITORS);
    for (uint8_t c = 0; c < NUM_CAPACITORS; c++)
        wr->cap[c].C_ind = C_ind_arr[c];

    /* ── ESR individual diff ── */
    float prev1=0, prev2=0, prev3=0;
    for (int c=(int)NUM_CAPACITORS-1; c>=0; c--){
        float d1 = ESR1_arr[c] - prev1;
        float d2 = ESR2_arr[c] - prev2;
        float d3 = ESR3_arr[c] - prev3;
        prev1 = ESR1_arr[c]; prev2 = ESR2_arr[c]; prev3 = ESR3_arr[c];
        wr->cap[c].ESR_I1 = d1;
        wr->cap[c].ESR_I2 = d2;
        wr->cap[c].ESR_I3 = d3;
    }

    /* ── Individual slope from C_ind ── */
    for (uint8_t c = 0; c < NUM_CAPACITORS; c++) {
        CapacitorResult_t *r = &wr->cap[c];
        r->slope_ind = (r->C_ind > 1e-9f && r->I2_avg > 1e-9f)
                       ? -(r->I2_avg / r->C_ind) : 0.0f;
    }

    wr->complete = true;
    return true;
}


void Analysis_Init(AnalysisEngine_t *e, uint16_t threshold_low_mv)
{
    memset(e, 0, sizeof(AnalysisEngine_t));
    e->threshold_low_mv = threshold_low_mv;
}

bool Analysis_AddSample(AnalysisEngine_t *e, uint32_t ts_ms,
                        const int16_t v[NUM_CAPACITORS],
                        float I1, float I2, float I3)
{
    if (e->sample_count >= MAX_SAMPLES){ e->threshold_reached=true; return false; }

    uint16_t idx = e->sample_count;
    e->samples[idx].timestamp_us = ts_ms;
    e->samples[idx].current_i1   = I1;
    e->samples[idx].current_i2   = I2;
    e->samples[idx].current_i3   = I3;
    for (uint8_t i=0;i<NUM_CAPACITORS;i++) e->samples[idx].voltage[i] = v[i];
    e->sample_count++;

    if (e->threshold_low_mv > 0){
        for (uint8_t i=0;i<NUM_CAPACITORS;i++){
            if (v[i] < (int16_t)e->threshold_low_mv){
                e->threshold_reached = true;
                return false;
            }
        }
    }
    return true;
}

uint8_t Analysis_DetectAllWindows(AnalysisEngine_t *e)
{
    return DetectWindows(e);
}

bool Analysis_PerformAllRegressions(AnalysisEngine_t *e)
{
    if (e->window_count == 0) return false;
    e->results_count = 0;
    for (uint8_t w=0; w<e->window_count && w<MAX_WINDOWS; w++)
        if (AnalyseWindow(e, &e->detected_windows[w], &e->results[w]))
            e->results_count++;
    e->analysis_complete = (e->results_count > 0);
    return e->analysis_complete;
}

uint16_t Analysis_FormatResults(AnalysisEngine_t *e, char *buf, uint16_t bufsz)
{
    if (!e->analysis_complete || !buf || bufsz==0) return 0;
    uint16_t pos=0; int left=(int)bufsz;

#define PR(fmt,...) do{ \
    int _n=snprintf(buf+pos,(size_t)left,fmt,##__VA_ARGS__); \
    if(_n<0||_n>=left)return pos; \
    pos+=(uint16_t)_n; left-=_n; }while(0)

    PR("\r\n=== CAPACITANCE & ESR: %u window(s) ===\r\n",(unsigned)e->results_count);

    for (uint8_t w=0; w<e->results_count; w++){
        WindowResult_t *wr = &e->results[w];
        if (!wr->complete) continue;

        float t_rs_c = ((float)wr->win.t_start_ms + (float)CAP_REG_START_OFFSET_MS);
        float t_re_c = (float)wr->win.t_end_ms * CAP_REG_END_FACTOR;
        if (t_re_c <= t_rs_c) t_re_c = (float)wr->win.t_end_ms;
        float duration_ms = (float)(wr->win.t_end_ms - wr->win.t_start_ms);

        float t_rs_r = (float)(wr->win.t_start_ms + ESR_START_OFFSET_MS);
        float t_re_r = (float)(wr->win.t_start_ms + ESR_END_OFFSET_MS);
        if (t_re_r > (float)wr->win.t_end_ms)
            t_re_r = (float)wr->win.t_end_ms;

        PR("\r\n--- Window %u: [%s]  Time=(%.3f,%.3f)ms  Duration=%.3fms ---\r\n",
           (unsigned)(w+1), wr->win.type,
           (float)wr->win.t_start_ms,
           (float)wr->win.t_end_ms,
           duration_ms);
        PR("    Cap window: (%.3f, %.3f)ms  Duration:%.3fms  Step:%dms\r\n",
           t_rs_c, t_re_c, t_re_c - t_rs_c, SLIDE_STEP_MS);
        PR("    ESR window: (%.3f, %.3f)ms  Duration:%.3fms  t_predict=%.3fms\r\n",
           t_rs_r, t_re_r, t_re_r - t_rs_r, wr->cap[0].t_predict_abs*1000.0f);
        PR("------------------------------------------------------------------------------------------------------------------------------\r\n");
        PR("Ch |  V_pre  | V_pred  | Diff. V |    C    |  C(ind) |  Slope   | ESR_I1 | ESR_I2 | ESR_I3 | R2_C  | RMSE_C |R2_ESR |RMSE_ESR|\r\n");
        PR("   |   (V)   |   (V)   |   (V)   |   (F)   |   (F)   |  (mV/s)  | (mOhm) | (mOhm) | (mOhm) |       |  (mV)  |       |  (mV)  |\r\n");
        PR("---+---------+---------+---------+---------+---------+----------+--------+--------+--------+-------+--------+-------+---------\r\n");

        for (uint8_t c = 0; c < NUM_CAPACITORS; c++) {

            CapacitorResult_t *r = &wr->cap[c];

            float v_diff[NUM_CAPACITORS] = {0};

            for (uint8_t i = 0; i + 1 < NUM_CAPACITORS; i++) {
                v_diff[i] = wr->cap[i].V_pre - wr->cap[i + 1].V_pre;
            }
            v_diff[NUM_CAPACITORS - 1] = wr->cap[7].V_pre;
            PR("V%u | %7.4f | %7.4f | %7.4f | %7.2f | %7.2f | %8.3f | %6.1f | %6.1f | %6.1f | %5.3f | %6.1f | %5.3f | %6.1f |\r\n",
               (unsigned)(c + 1),
               r->V_pre,
               r->V_pred,
			   v_diff[c],
               r->C_ind,
               r->C_ind / 4.0f,
               r->slope_ind * 1000.0f,
               r->ESR_I1,
               r->ESR_I2,
               r->ESR_I3,
               r->r2_C,
               r->rmse_c_mv,
               r->r2_esr,
               r->rmse_esr_mv
            );
        }
        PR("------------------------------------------------------------------------------------------------------------------------------\r\n");

        float avgC=0, cntC=0, totE1=0, totE2=0, totE3=0;
        float sumI1=0, sumI2=0, sumI3=0;
        float cntI1=0, cntI2=0, cntI3=0;
        for (uint8_t c=0; c<NUM_CAPACITORS; c++){
            if (wr->cap[c].C_ind > 0 && !_isnan(wr->cap[c].C_ind))
                { avgC += wr->cap[c].C_ind; cntC++; }
            totE1 += wr->cap[c].ESR_I1;
            totE2 += wr->cap[c].ESR_I2;
            totE3 += wr->cap[c].ESR_I3;
            if (wr->cap[c].I1_avg > 1e-9f){ sumI1 += wr->cap[c].I1_avg; cntI1++; }
            if (wr->cap[c].I2_avg > 1e-9f){ sumI2 += wr->cap[c].I2_avg; cntI2++; }
            if (wr->cap[c].I3_avg > 1e-9f){ sumI3 += wr->cap[c].I3_avg; cntI3++; }
        }
        if (cntC > 0) avgC /= cntC;
        float avgI1 = (cntI1>0) ? sumI1/cntI1 : 0.0f;
        float avgI2 = (cntI2>0) ? sumI2/cntI2 : 0.0f;
        float avgI3 = (cntI3>0) ? sumI3/cntI3 : 0.0f;

        PR("----- SUMMARY W%u -----\r\n",(unsigned)(w+1));
        PR("Avg_C (Parallel)    : %.2f F\r\n", avgC);
        PR("Avg_C_ind(Parallel) : %.2f F\r\n", avgC/4.0f);
        PR("Total_C             : %.2f F\r\n", wr->cap[0].C_eq);
        PR("Total ESR_I1        : %.2f mOhm\r\n", totE1);
        PR("Total ESR_I2        : %.2f mOhm\r\n", totE2);
        PR("Total ESR_I3        : %.2f mOhm\r\n", totE3);
        PR("Avg_I1              : %.4f A\r\n", avgI1);
        PR("Avg_I2              : %.4f A\r\n", avgI2);
        PR("Avg_I3              : %.4f A\r\n", avgI3);
    }
    PR("\r\n----- Done -----\r\n");
#undef PR
    return pos;
}

void Analysis_Reset(AnalysisEngine_t *e)
{
    uint16_t thr = e->threshold_low_mv;
    memset(e, 0, sizeof(AnalysisEngine_t));
    e->threshold_low_mv = thr;
}
