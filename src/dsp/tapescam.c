/*
 * TAPESCAM Audio FX Plugin
 *
 * Tape saturation and degradation effect with:
 * - Drive: Input gain
 * - Saturation: Soft clipping via tanh waveshaping
 * - Wobble: LFO-modulated pitch variation (wow/flutter)
 * - Tone: One-pole lowpass filter
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "audio_fx_api_v1.h"

#define SAMPLE_RATE 44100

/* Delay line for wow/flutter (~50ms at 44100Hz) */
#define DELAY_SIZE 2205

/* Plugin state */
static const host_api_v1_t *g_host = NULL;
static audio_fx_api_v1_t g_fx_api;

/* Parameters (0.0 to 1.0) */
static float g_drive = 0.3f;
static float g_saturation = 0.5f;
static float g_wobble = 0.2f;
static float g_tone = 0.5f;

/* Delay line for wobble effect */
static float g_delay_l[DELAY_SIZE];
static float g_delay_r[DELAY_SIZE];
static int g_delay_idx = 0;

/* LFO state */
static float g_lfo_phase_wow = 0.0f;     /* Slow wow LFO (~0.5Hz) */
static float g_lfo_phase_flutter = 0.0f; /* Fast flutter LFO (~6Hz) */

/* Tone filter state (one-pole lowpass) */
static float g_filter_l = 0.0f;
static float g_filter_r = 0.0f;

/* Logging helper */
static void fx_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[tapescam] %s", msg);
        g_host->log(buf);
    }
}

/* Soft clipping via normalized tanh */
static inline float saturate(float x, float amount) {
    float drive_amount = 1.0f + amount * 3.0f;  /* 1x to 4x */
    float tanh_drive = tanhf(drive_amount);
    if (tanh_drive < 0.001f) tanh_drive = 0.001f;  /* Avoid division by zero */
    return tanhf(x * drive_amount) / tanh_drive;
}

/* Linear interpolation for fractional delay read */
static inline float delay_read(float *delay, int idx, float frac_delay) {
    /* frac_delay is 0.0 to DELAY_SIZE-1 */
    int i0 = (int)frac_delay;
    int i1 = i0 + 1;
    if (i1 >= DELAY_SIZE) i1 = 0;

    float frac = frac_delay - (float)i0;

    /* Calculate read positions (going backwards from write position) */
    int read0 = idx - i0;
    int read1 = idx - i1;
    if (read0 < 0) read0 += DELAY_SIZE;
    if (read1 < 0) read1 += DELAY_SIZE;

    return delay[read0] * (1.0f - frac) + delay[read1] * frac;
}

/* === Audio FX API Implementation === */

static int fx_on_load(const char *module_dir, const char *config_json) {
    char msg[256];
    snprintf(msg, sizeof(msg), "TAPESCAM loading from: %s", module_dir);
    fx_log(msg);

    /* Clear delay lines */
    memset(g_delay_l, 0, sizeof(g_delay_l));
    memset(g_delay_r, 0, sizeof(g_delay_r));
    g_delay_idx = 0;

    /* Reset LFO phases */
    g_lfo_phase_wow = 0.0f;
    g_lfo_phase_flutter = 0.0f;

    /* Reset filter state */
    g_filter_l = 0.0f;
    g_filter_r = 0.0f;

    fx_log("TAPESCAM initialized");
    return 0;
}

static void fx_on_unload(void) {
    fx_log("TAPESCAM unloading");
}

static void fx_process_block(int16_t *audio_inout, int frames) {
    /* LFO frequencies */
    const float wow_freq = 0.5f;     /* 0.5 Hz */
    const float flutter_freq = 6.0f; /* 6 Hz */
    const float dt = 1.0f / SAMPLE_RATE;

    /* Calculate tone filter coefficient */
    /* Cutoff from 2kHz (tone=0) to 20kHz (tone=1) */
    float freq = 2000.0f + g_tone * 18000.0f;
    float rc = 1.0f / (2.0f * M_PI * freq);
    float alpha = dt / (rc + dt);

    /* Drive amount: 1.0 to 4.0x */
    float drive_gain = 1.0f + g_drive * 3.0f;

    /* Wobble modulation depth (in samples, up to ~10ms) */
    float wobble_depth = g_wobble * 441.0f;  /* ~10ms at 44100Hz */

    /* Base delay (center point for modulation, ~25ms) */
    float base_delay = 1102.0f;

    for (int i = 0; i < frames; i++) {
        /* Convert to float (-1.0 to 1.0) */
        float in_l = audio_inout[i * 2] / 32768.0f;
        float in_r = audio_inout[i * 2 + 1] / 32768.0f;

        /* Stage 1: Input gain (Drive) */
        float driven_l = in_l * drive_gain;
        float driven_r = in_r * drive_gain;

        /* Stage 2: Saturation */
        float sat_l = saturate(driven_l, g_saturation);
        float sat_r = saturate(driven_r, g_saturation);

        /* Write to delay line */
        g_delay_l[g_delay_idx] = sat_l;
        g_delay_r[g_delay_idx] = sat_r;

        /* Stage 3: Wobble (wow/flutter via modulated delay) */
        float wobble_l, wobble_r;
        if (g_wobble > 0.001f) {
            /* Calculate LFO modulation (combine wow and flutter) */
            float wow = sinf(g_lfo_phase_wow * 2.0f * M_PI);
            float flutter = sinf(g_lfo_phase_flutter * 2.0f * M_PI) * 0.3f;
            float mod = (wow + flutter) * wobble_depth;

            /* Calculate modulated delay time */
            float delay_time = base_delay + mod;
            if (delay_time < 1.0f) delay_time = 1.0f;
            if (delay_time > DELAY_SIZE - 2) delay_time = DELAY_SIZE - 2;

            /* Read from delay with interpolation */
            wobble_l = delay_read(g_delay_l, g_delay_idx, delay_time);
            wobble_r = delay_read(g_delay_r, g_delay_idx, delay_time);

            /* Update LFO phases */
            g_lfo_phase_wow += wow_freq * dt;
            if (g_lfo_phase_wow >= 1.0f) g_lfo_phase_wow -= 1.0f;
            g_lfo_phase_flutter += flutter_freq * dt;
            if (g_lfo_phase_flutter >= 1.0f) g_lfo_phase_flutter -= 1.0f;
        } else {
            /* No wobble - pass through saturated signal */
            wobble_l = sat_l;
            wobble_r = sat_r;
        }

        /* Advance delay write index */
        g_delay_idx++;
        if (g_delay_idx >= DELAY_SIZE) g_delay_idx = 0;

        /* Stage 4: Tone filter (one-pole lowpass) */
        g_filter_l = g_filter_l + alpha * (wobble_l - g_filter_l);
        g_filter_r = g_filter_r + alpha * (wobble_r - g_filter_r);

        float out_l = g_filter_l;
        float out_r = g_filter_r;

        /* Clamp and convert back to int16 */
        if (out_l > 1.0f) out_l = 1.0f;
        if (out_l < -1.0f) out_l = -1.0f;
        if (out_r > 1.0f) out_r = 1.0f;
        if (out_r < -1.0f) out_r = -1.0f;

        audio_inout[i * 2] = (int16_t)(out_l * 32767.0f);
        audio_inout[i * 2 + 1] = (int16_t)(out_r * 32767.0f);
    }
}

static void fx_set_param(const char *key, const char *val) {
    float v = atof(val);

    /* Clamp to 0.0-1.0 range */
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;

    if (strcmp(key, "drive") == 0) {
        g_drive = v;
    } else if (strcmp(key, "saturation") == 0) {
        g_saturation = v;
    } else if (strcmp(key, "wobble") == 0) {
        g_wobble = v;
    } else if (strcmp(key, "tone") == 0) {
        g_tone = v;
    }
}

static int fx_get_param(const char *key, char *buf, int buf_len) {
    if (strcmp(key, "drive") == 0) {
        return snprintf(buf, buf_len, "%.2f", g_drive);
    } else if (strcmp(key, "saturation") == 0) {
        return snprintf(buf, buf_len, "%.2f", g_saturation);
    } else if (strcmp(key, "wobble") == 0) {
        return snprintf(buf, buf_len, "%.2f", g_wobble);
    } else if (strcmp(key, "tone") == 0) {
        return snprintf(buf, buf_len, "%.2f", g_tone);
    } else if (strcmp(key, "name") == 0) {
        return snprintf(buf, buf_len, "TAPESCAM");
    }
    return -1;
}

/* === Entry Point === */

audio_fx_api_v1_t* move_audio_fx_init_v1(const host_api_v1_t *host) {
    g_host = host;

    memset(&g_fx_api, 0, sizeof(g_fx_api));
    g_fx_api.api_version = AUDIO_FX_API_VERSION;
    g_fx_api.on_load = fx_on_load;
    g_fx_api.on_unload = fx_on_unload;
    g_fx_api.process_block = fx_process_block;
    g_fx_api.set_param = fx_set_param;
    g_fx_api.get_param = fx_get_param;

    fx_log("TAPESCAM plugin initialized");

    return &g_fx_api;
}
