/*
 * TAPESCAM Audio FX Plugin - Authentic Tape Degradation
 *
 * Port from TapeScamVST - CVCHothouse/TapeScam shared_dsp
 *
 * VST Signal Chain:
 * Input Level -> GainStage (drive/clipping) -> TapeSat -> WowFlutter -> Tone -> Output Level
 *
 * 6 Main Knobs (mapped to Move knobs 2-7):
 * Knob 2: INPUT  - Input level (pre-gain)
 * Knob 3: DRIVE  - GainStage drive amount
 * Knob 4: COLOR  - Saturation character (affects both GainStage and TapeSat)
 * Knob 5: WOBBLE - Wow/Flutter amount
 * Knob 6: TONE   - Tilt EQ (dark to bright)
 * Knob 7: OUTPUT - Output level
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "audio_fx_api_v1.h"

#define SAMPLE_RATE 48000.0f
#define kTwoPi 6.283185307179586476925286766559f
#define kPi 3.14159265358979323846f

static inline float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float dBToLin(float dB) {
    return powf(10.0f, dB / 20.0f);
}

/* ============================================================================
 * GAIN STAGE MODULE - Port from GainStageModule
 *
 * This is the main distortion engine with multi-stage clipping
 * ============================================================================ */

/* Biquad filter state for pre/post filtering */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} GS_Biquad;

static void GS_BiquadSetHighpass(GS_Biquad *bq, float sampleRate, float freq, float q) {
    float w0 = kTwoPi * freq / sampleRate;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * q);

    float b0 = (1.0f + cosw0) * 0.5f;
    float b1 = -(1.0f + cosw0);
    float b2 = (1.0f + cosw0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 = 1.0f - alpha;

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
}

static void GS_BiquadSetLowpass(GS_Biquad *bq, float sampleRate, float freq, float q) {
    float w0 = kTwoPi * freq / sampleRate;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * q);

    float b0 = (1.0f - cosw0) * 0.5f;
    float b1 = 1.0f - cosw0;
    float b2 = (1.0f - cosw0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 = 1.0f - alpha;

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
}

static float GS_BiquadProcess(GS_Biquad *bq, float x) {
    float y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}


/* ============================================================================
 * WOW/FLUTTER MODULE - Constants and shared utilities
 * ============================================================================ */

#define kMaxDelaySamples 2048
#define kBaseDelayMsL 5.0f
#define kBaseDelayMsR 5.5f

static float WF_InterpolateCubic(const float *buf, size_t size, float index) {
    int idx1 = (int)floorf(index);
    float frac = index - (float)idx1;
    int idx0 = idx1 - 1;
    int idx2 = idx1 + 1;
    int idx3 = idx1 + 2;

    while (idx0 < 0) idx0 += (int)size;
    while (idx1 < 0) idx1 += (int)size;
    while (idx2 < 0) idx2 += (int)size;
    while (idx3 < 0) idx3 += (int)size;
    idx0 %= (int)size;
    idx1 %= (int)size;
    idx2 %= (int)size;
    idx3 %= (int)size;

    float y0 = buf[idx0], y1 = buf[idx1], y2 = buf[idx2], y3 = buf[idx3];
    float c0 = y1;
    float c1 = 0.5f * (y2 - y0);
    float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return ((c3 * frac + c2) * frac + c1) * frac + c0;
}

/* ============================================================================
 * TONE MODULE - Shared utilities
 * ============================================================================ */

typedef struct {
    float b0, b1, b2, a1, a2, z1, z2;
} ToneBiquad;

static void Tone_ComputeShelf(ToneBiquad *state, float gainDb, float cutoffHz, int highShelf, float sampleRate) {
    float A = powf(10.0f, gainDb / 40.0f);
    float w0 = kTwoPi * cutoffHz / sampleRate;
    float cosw0 = cosf(w0), sinw0 = sinf(w0);
    float sqrtA = sqrtf(A);
    float alpha = sinw0 / 2.0f * sqrtf(2.0f);

    float b0, b1, b2, a0, a1, a2;
    if (!highShelf) {
        b0 = A * ((A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha);
        b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosw0);
        b2 = A * ((A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha);
        a0 = (A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha;
        a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosw0);
        a2 = (A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha;
    } else {
        b0 = A * ((A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha);
        b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosw0);
        b2 = A * ((A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha);
        a0 = (A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtA * alpha;
        a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cosw0);
        a2 = (A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtA * alpha;
    }

    float invA0 = 1.0f / a0;
    state->b0 = b0 * invA0;
    state->b1 = b1 * invA0;
    state->b2 = b2 * invA0;
    state->a1 = a1 * invA0;
    state->a2 = a2 * invA0;
}

static inline float Tone_ProcessBiquad(ToneBiquad *bq, float x) {
    float y = x * bq->b0 + bq->z1;
    bq->z1 = x * bq->b1 + bq->z2 - bq->a1 * y;
    bq->z2 = x * bq->b2 - bq->a2 * y;
    return y;
}

/* ============================================================================
 * V2 API - Instance-based (V1 API removed)
 * ============================================================================ */

static const host_api_v1_t *g_host = NULL;

#define AUDIO_FX_API_VERSION_2 2
#define AUDIO_FX_INIT_V2_SYMBOL "move_audio_fx_init_v2"

typedef struct audio_fx_api_v2 {
    uint32_t api_version;
    void* (*create_instance)(const char *module_dir, const char *config_json);
    void (*destroy_instance)(void *instance);
    void (*process_block)(void *instance, int16_t *audio_inout, int frames);
    void (*set_param)(void *instance, const char *key, const char *val);
    int (*get_param)(void *instance, const char *key, char *buf, int buf_len);
} audio_fx_api_v2_t;

typedef audio_fx_api_v2_t* (*audio_fx_init_v2_fn)(const host_api_v1_t *host);

/* Instance structure for v2 API */
typedef struct {
    char module_dir[256];

    /* User parameters (0-1) */
    float param_input;
    float param_drive;
    float param_color;
    float param_wobble;
    float param_tone;
    float param_output;

    /* GainStage state */
    float gs_trimGainLin;
    float gs_channelGainLin;
    float gs_masterVolLin;
    float gs_driveNorm;
    float gs_character;
    float gs_characterDriveScale;
    float gs_stage1Softness;
    float gs_stage2Softness;
    float gs_stage2Asymmetry;
    GS_Biquad gs_preHP[2];
    GS_Biquad gs_preLP[2];
    GS_Biquad gs_postLP[2];

    /* TapeSat state */
    float sat_targetDrive;
    float sat_smoothedDrive;
    float sat_factor;

    /* WowFlutter state */
    float *delayBufL;
    float *delayBufR;
    size_t writeIndex;
    float wf_targetAmount;
    float wf_smoothedAmount;
    float wf_activationRamp;
    float wf_crossfadeGain;
    float wf_depthShape;
    float wf_slewAlpha;
    float wf_wowAmount;
    float wf_flutterAmount;
    float wf_wowRateHz;
    float wf_flutterRateHz;
    float wf_wowPhaseL;
    float wf_wowPhaseR;
    float wf_fltPhaseL;
    float wf_fltPhaseR;
    float wf_readStateL;
    float wf_readStateR;
    float wf_prevReadPosL;
    float wf_prevReadPosR;
    float wf_baseDelaySamplesL;
    float wf_baseDelaySamplesR;
    float wf_wowNoiseStateL;
    float wf_wowNoiseStateR;
    float wf_flutterNoiseStateL;
    float wf_flutterNoiseStateR;
    uint32_t randState;

    /* Tone state */
    ToneBiquad tone_lowShelfL;
    ToneBiquad tone_lowShelfR;
    ToneBiquad tone_highShelfL;
    ToneBiquad tone_highShelfR;
    float tone_targetAmount;
    float tone_smoothedAmount;
    float tone_bassGainDb;
    float tone_trebleGainDb;
} tapescam_instance_t;

static void v2_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[tapescam-v2] %s", msg);
        g_host->log(buf);
    }
}

/* Instance-based DSP helpers */
static void v2_GS_Init(tapescam_instance_t *inst, float sampleRate) {
    inst->gs_trimGainLin = 1.0f;
    inst->gs_channelGainLin = 1.0f;
    inst->gs_masterVolLin = 1.0f;
    inst->gs_driveNorm = 0.0f;
    inst->gs_character = 0.0f;
    inst->gs_characterDriveScale = 1.0f;
    inst->gs_stage1Softness = 1.0f;
    inst->gs_stage2Softness = 1.2f;
    inst->gs_stage2Asymmetry = 0.05f;

    float preHpCutHz = 160.0f * 0.9f;
    float preLpCutHz = 5200.0f * 1.1f;
    float postLpCutHz = 17500.0f;

    for (int ch = 0; ch < 2; ch++) {
        memset(&inst->gs_preHP[ch], 0, sizeof(GS_Biquad));
        memset(&inst->gs_preLP[ch], 0, sizeof(GS_Biquad));
        memset(&inst->gs_postLP[ch], 0, sizeof(GS_Biquad));
        inst->gs_preHP[ch].b0 = 1.0f;
        inst->gs_preLP[ch].b0 = 1.0f;
        inst->gs_postLP[ch].b0 = 1.0f;

        GS_BiquadSetHighpass(&inst->gs_preHP[ch], sampleRate, preHpCutHz, 0.7071f);
        GS_BiquadSetLowpass(&inst->gs_preLP[ch], sampleRate, preLpCutHz, 0.7071f);
        GS_BiquadSetLowpass(&inst->gs_postLP[ch], sampleRate, postLpCutHz, 0.7071f);
    }
}

static void v2_GS_SetParams(tapescam_instance_t *inst, float drive, float color) {
    float driveNorm = Clamp(drive, 0.0f, 1.0f);
    float driveShaped = driveNorm * driveNorm;
    float colorNorm = Clamp(color, 0.0f, 1.0f);

    float trimDb = -2.0f + driveShaped * 34.0f;
    float channelDb = -2.0f + driveNorm * 26.0f;

    inst->gs_trimGainLin = dBToLin(trimDb);
    inst->gs_channelGainLin = dBToLin(channelDb);
    inst->gs_driveNorm = driveNorm;
    inst->gs_character = colorNorm;
    inst->gs_characterDriveScale = 0.8f + 1.2f * colorNorm;
    inst->gs_stage1Softness = 0.8f + 0.4f * (1.0f - colorNorm);
    inst->gs_stage2Softness = 1.05f + 1.1f * colorNorm;
    inst->gs_stage2Asymmetry = 0.02f + 0.08f * colorNorm;
}

static void v2_GS_SetOutputLevel(tapescam_instance_t *inst, float level) {
    float levelDb = -12.0f + level * 18.0f;
    inst->gs_masterVolLin = dBToLin(levelDb);
}

static float v2_GS_ApplyClipping(tapescam_instance_t *inst, float x, float softness) {
    float drive = 1.0f + inst->gs_driveNorm * inst->gs_characterDriveScale;
    float v = x * drive;
    v = tanhf(v * softness);
    float asymBase = 0.01f + 0.03f * inst->gs_character;
    v += asymBase * v * v * (v >= 0.0f ? 1.0f : -1.0f);
    return v;
}

static void v2_GS_Process(tapescam_instance_t *inst, float *left, float *right, int size) {
    for (int i = 0; i < size; i++) {
        float stage0L = left[i];
        float stage0R = right[i];

        float stage1InL = stage0L * inst->gs_trimGainLin;
        float stage1InR = stage0R * inst->gs_trimGainLin;
        float stage1OutL = v2_GS_ApplyClipping(inst, stage1InL, inst->gs_stage1Softness);
        float stage1OutR = v2_GS_ApplyClipping(inst, stage1InR, inst->gs_stage1Softness);

        float preEqL = GS_BiquadProcess(&inst->gs_preLP[0], GS_BiquadProcess(&inst->gs_preHP[0], stage1OutL));
        float preEqR = GS_BiquadProcess(&inst->gs_preLP[1], GS_BiquadProcess(&inst->gs_preHP[1], stage1OutR));

        float stage2InL = preEqL * inst->gs_channelGainLin;
        float stage2InR = preEqR * inst->gs_channelGainLin;
        float stage2OutL = v2_GS_ApplyClipping(inst, stage2InL, inst->gs_stage2Softness);
        float stage2OutR = v2_GS_ApplyClipping(inst, stage2InR, inst->gs_stage2Softness);
        float asym0 = inst->gs_stage2Asymmetry;
        float asym1 = -inst->gs_stage2Asymmetry;
        stage2OutL += asym0 * stage2OutL * stage2OutL;
        stage2OutR += asym1 * stage2OutR * stage2OutR;

        float postL = GS_BiquadProcess(&inst->gs_postLP[0], stage2OutL);
        float postR = GS_BiquadProcess(&inst->gs_postLP[1], stage2OutR);

        left[i] = Clamp(postL * inst->gs_masterVolLin, -0.98f, 0.98f);
        right[i] = Clamp(postR * inst->gs_masterVolLin, -0.98f, 0.98f);
    }
}

static void v2_TapeSat_SetDrive(tapescam_instance_t *inst, float normalizedDrive) {
    float shaped = Clamp(normalizedDrive, 0.0f, 1.0f);
    shaped = powf(shaped, 0.8f);
    inst->sat_targetDrive = shaped;
}

static void v2_TapeSat_UpdateControls(tapescam_instance_t *inst) {
    float delta = inst->sat_targetDrive - inst->sat_smoothedDrive;
    inst->sat_smoothedDrive += delta * 0.3f;
    if (fabsf(inst->sat_targetDrive - inst->sat_smoothedDrive) < 1e-4f) {
        inst->sat_smoothedDrive = inst->sat_targetDrive;
    }
    inst->sat_factor = Clamp(inst->sat_smoothedDrive * 1.35f, 0.0f, 2.0f);
}

static void v2_TapeSat_Process(tapescam_instance_t *inst, float *left, float *right, int size) {
    if (inst->sat_smoothedDrive < 0.01f) return;
    float satAmount = inst->sat_factor;
    if (satAmount < 0.01f) return;

    float drive = 1.0f + satAmount * 2.5f;
    for (int i = 0; i < size; i++) {
        left[i] = tanhf(left[i] * drive);
        right[i] = tanhf(right[i] * drive);
    }
}

static float v2_WF_NextRand(tapescam_instance_t *inst) {
    inst->randState = inst->randState * 1664525u + 1013904223u;
    return (float)inst->randState * 2.3283064365386963e-10f;
}

static float v2_WF_NextRandCentered(tapescam_instance_t *inst) {
    return v2_WF_NextRand(inst) * 2.0f - 1.0f;
}

static void v2_WF_Init(tapescam_instance_t *inst, float sampleRate) {
    inst->delayBufL = (float*)calloc(kMaxDelaySamples, sizeof(float));
    inst->delayBufR = (float*)calloc(kMaxDelaySamples, sizeof(float));
    inst->writeIndex = 0;

    inst->wf_baseDelaySamplesL = fminf(sampleRate * (kBaseDelayMsL * 0.001f), (float)kMaxDelaySamples - 4.0f);
    inst->wf_baseDelaySamplesR = fminf(sampleRate * (kBaseDelayMsR * 0.001f), (float)kMaxDelaySamples - 4.0f);
    inst->wf_readStateL = inst->wf_baseDelaySamplesL;
    inst->wf_readStateR = inst->wf_baseDelaySamplesR;

    inst->wf_targetAmount = 0.0f;
    inst->wf_smoothedAmount = 0.0f;
    inst->wf_activationRamp = 0.0f;
    inst->wf_crossfadeGain = 0.0f;
    inst->wf_depthShape = 0.0f;
    inst->wf_slewAlpha = 0.9f;
    inst->wf_wowAmount = 0.0f;
    inst->wf_flutterAmount = 0.0f;
    inst->wf_wowRateHz = 0.3f;
    inst->wf_flutterRateHz = 4.0f;
    inst->wf_wowPhaseL = 0.0f;
    inst->wf_wowPhaseR = 0.25f;
    inst->wf_fltPhaseL = 0.0f;
    inst->wf_fltPhaseR = 0.33f;
    inst->wf_wowNoiseStateL = 0.0f;
    inst->wf_wowNoiseStateR = 0.0f;
    inst->wf_flutterNoiseStateL = 0.0f;
    inst->wf_flutterNoiseStateR = 0.0f;

    inst->randState ^= (uint32_t)sampleRate;
}

static void v2_WF_SetAmount(tapescam_instance_t *inst, float amount) {
    inst->wf_targetAmount = Clamp(amount, 0.0f, 1.0f);
}

static void v2_WF_UpdateControls(tapescam_instance_t *inst) {
    inst->wf_smoothedAmount += (inst->wf_targetAmount - inst->wf_smoothedAmount) * 0.3f;
    float targetActive = (inst->wf_targetAmount > 0.0005f) ? 1.0f : 0.0f;
    inst->wf_activationRamp += (targetActive - inst->wf_activationRamp) * 0.4f;

    float amt = inst->wf_smoothedAmount * inst->wf_activationRamp;
    inst->wf_depthShape = amt * amt;

    float depthBlendWow = 0.05f + 0.95f * inst->wf_depthShape;
    float depthBlendFlutter = 0.03f + 0.97f * inst->wf_depthShape;

    float targetSlew = (amt <= 0.0005f) ? 0.97f : Clamp(0.94f - 0.28f * inst->wf_depthShape, 0.7f, 0.95f);
    inst->wf_slewAlpha += (targetSlew - inst->wf_slewAlpha) * 0.2f;

    inst->wf_wowAmount = depthBlendWow * inst->wf_activationRamp;
    inst->wf_flutterAmount = depthBlendFlutter * inst->wf_activationRamp;
}

static void v2_WF_Process(tapescam_instance_t *inst, float *inL, float *inR, float *outL, float *outR, int size, float sampleRate) {
    if (!inst->delayBufL || !inst->delayBufR) {
        for (int i = 0; i < size; i++) {
            outL[i] = inL[i];
            outR[i] = inR[i];
        }
        return;
    }

    float wowAmt = inst->wf_wowAmount;
    float flutterAmt = inst->wf_flutterAmount;
    float invSr = 1.0f / sampleRate;

    int bypassDelay = (inst->wf_activationRamp < 1.0e-4f && wowAmt <= 1.0e-5f && flutterAmt <= 1.0e-5f);
    if (bypassDelay) {
        for (int i = 0; i < size; i++) {
            outL[i] = inL[i];
            outR[i] = inR[i];
            inst->delayBufL[inst->writeIndex] = inL[i];
            inst->delayBufR[inst->writeIndex] = inR[i];
            inst->writeIndex++;
            if (inst->writeIndex >= kMaxDelaySamples) inst->writeIndex = 0;
        }
        return;
    }

    float crossfadeAlpha = 0.004f;
    float crossfadeGain = inst->wf_crossfadeGain;
    float maxWowDepthSec = 0.035f;
    float maxFlutterDepthSec = 0.002f;
    float maxDevSec = 0.0042f;

    for (int i = 0; i < size; i++) {
        float xL = inL[i], xR = inR[i];
        crossfadeGain += (inst->wf_activationRamp - crossfadeGain) * crossfadeAlpha;

        inst->wf_wowNoiseStateL += (v2_WF_NextRandCentered(inst) - inst->wf_wowNoiseStateL) * 0.00018f;
        inst->wf_wowNoiseStateR += (v2_WF_NextRandCentered(inst) - inst->wf_wowNoiseStateR) * 0.00018f;
        inst->wf_flutterNoiseStateL += (v2_WF_NextRandCentered(inst) - inst->wf_flutterNoiseStateL) * 0.00025f;
        inst->wf_flutterNoiseStateR += (v2_WF_NextRandCentered(inst) - inst->wf_flutterNoiseStateR) * 0.00025f;

        inst->wf_wowPhaseL += inst->wf_wowRateHz * invSr;
        if (inst->wf_wowPhaseL >= 1.0f) inst->wf_wowPhaseL -= 1.0f;
        inst->wf_fltPhaseL += inst->wf_flutterRateHz * invSr;
        if (inst->wf_fltPhaseL >= 1.0f) inst->wf_fltPhaseL -= 1.0f;

        inst->wf_wowPhaseR += inst->wf_wowRateHz * 1.03f * invSr;
        if (inst->wf_wowPhaseR >= 1.0f) inst->wf_wowPhaseR -= 1.0f;
        inst->wf_fltPhaseR += inst->wf_flutterRateHz * 0.97f * invSr;
        if (inst->wf_fltPhaseR >= 1.0f) inst->wf_fltPhaseR -= 1.0f;

        float depthSlow = sqrtf(fmaxf(inst->wf_depthShape, 0.0f));
        float wowNoiseMix = Clamp(0.4f * (0.10f + 0.45f * depthSlow), 0.0f, 0.65f);
        float flutterNoiseMix = Clamp(0.2f * (0.12f + 0.45f * depthSlow), 0.0f, 0.7f);

        float wowShapeL = (1.0f - wowNoiseMix) * sinf(kTwoPi * inst->wf_wowPhaseL) + wowNoiseMix * inst->wf_wowNoiseStateL;
        float flutterShapeL = (1.0f - flutterNoiseMix) * sinf(kTwoPi * inst->wf_fltPhaseL) + flutterNoiseMix * inst->wf_flutterNoiseStateL;
        float wowShapeR = (1.0f - wowNoiseMix) * sinf(kTwoPi * inst->wf_wowPhaseR) + wowNoiseMix * inst->wf_wowNoiseStateR;
        float flutterShapeR = (1.0f - flutterNoiseMix) * sinf(kTwoPi * inst->wf_fltPhaseR) + flutterNoiseMix * inst->wf_flutterNoiseStateR;

        float devSamplesL = (maxWowDepthSec * wowAmt * wowShapeL + maxFlutterDepthSec * flutterAmt * flutterShapeL) * sampleRate;
        float devSamplesR = (maxWowDepthSec * wowAmt * 1.03f * wowShapeR + maxFlutterDepthSec * flutterAmt * 0.97f * flutterShapeR) * sampleRate;

        float maxDevSamples = maxDevSec * sampleRate;
        devSamplesL = Clamp(devSamplesL, -maxDevSamples, maxDevSamples);
        devSamplesR = Clamp(devSamplesR, -maxDevSamples, maxDevSamples);

        float targetSamplesL = fmaxf(inst->wf_baseDelaySamplesL + devSamplesL, 0.0002f * sampleRate);
        float targetSamplesR = fmaxf(inst->wf_baseDelaySamplesR + devSamplesR, 0.0002f * sampleRate);

        inst->wf_readStateL = inst->wf_slewAlpha * inst->wf_readStateL + (1.0f - inst->wf_slewAlpha) * targetSamplesL;
        inst->wf_readStateR = inst->wf_slewAlpha * inst->wf_readStateR + (1.0f - inst->wf_slewAlpha) * targetSamplesR;

        float readIndexL = (float)inst->writeIndex - inst->wf_readStateL;
        float readIndexR = (float)inst->writeIndex - inst->wf_readStateR;
        if (readIndexL < 0.0f) readIndexL += (float)kMaxDelaySamples;
        if (readIndexR < 0.0f) readIndexR += (float)kMaxDelaySamples;

        float delayedL = WF_InterpolateCubic(inst->delayBufL, kMaxDelaySamples, readIndexL);
        float delayedR = WF_InterpolateCubic(inst->delayBufR, kMaxDelaySamples, readIndexR);

        inst->delayBufL[inst->writeIndex] = xL;
        inst->delayBufR[inst->writeIndex] = xR;
        inst->writeIndex++;
        if (inst->writeIndex >= kMaxDelaySamples) inst->writeIndex = 0;

        float wet = crossfadeGain, dry = 1.0f - wet;
        outL[i] = dry * xL + wet * delayedL;
        outR[i] = dry * xR + wet * delayedR;
    }
    inst->wf_crossfadeGain = crossfadeGain;
}

static void v2_Tone_Init(tapescam_instance_t *inst) {
    memset(&inst->tone_lowShelfL, 0, sizeof(ToneBiquad));
    memset(&inst->tone_lowShelfR, 0, sizeof(ToneBiquad));
    memset(&inst->tone_highShelfL, 0, sizeof(ToneBiquad));
    memset(&inst->tone_highShelfR, 0, sizeof(ToneBiquad));
    inst->tone_lowShelfL.b0 = inst->tone_lowShelfR.b0 = 1.0f;
    inst->tone_highShelfL.b0 = inst->tone_highShelfR.b0 = 1.0f;
    inst->tone_targetAmount = 0.5f;
    inst->tone_smoothedAmount = 0.5f;
}

static void v2_Tone_SetAmount(tapescam_instance_t *inst, float amount) {
    inst->tone_targetAmount = Clamp(amount, 0.0f, 1.0f);
}

static void v2_Tone_UpdateControls(tapescam_instance_t *inst, float sampleRate) {
    inst->tone_smoothedAmount += (inst->tone_targetAmount - inst->tone_smoothedAmount) * 0.5f;
    float normalized = inst->tone_smoothedAmount * 2.0f - 1.0f;
    float shaped = normalized * (1.0f + 0.9f * fabsf(normalized));
    inst->tone_bassGainDb = -shaped * 15.0f;
    inst->tone_trebleGainDb = shaped * 15.0f;

    Tone_ComputeShelf(&inst->tone_lowShelfL, inst->tone_bassGainDb, 120.0f, 0, sampleRate);
    Tone_ComputeShelf(&inst->tone_lowShelfR, inst->tone_bassGainDb, 120.0f, 0, sampleRate);
    Tone_ComputeShelf(&inst->tone_highShelfL, inst->tone_trebleGainDb, 6000.0f, 1, sampleRate);
    Tone_ComputeShelf(&inst->tone_highShelfR, inst->tone_trebleGainDb, 6000.0f, 1, sampleRate);
}

static void v2_Tone_Process(tapescam_instance_t *inst, float *bufL, float *bufR, int size) {
    for (int i = 0; i < size; i++) {
        float y = Tone_ProcessBiquad(&inst->tone_lowShelfL, bufL[i]);
        bufL[i] = Tone_ProcessBiquad(&inst->tone_highShelfL, y);
        y = Tone_ProcessBiquad(&inst->tone_lowShelfR, bufR[i]);
        bufR[i] = Tone_ProcessBiquad(&inst->tone_highShelfR, y);
    }
}

static void* v2_create_instance(const char *module_dir, const char *config_json) {
    v2_log("Creating instance");

    tapescam_instance_t *inst = (tapescam_instance_t*)calloc(1, sizeof(tapescam_instance_t));
    if (!inst) {
        v2_log("Failed to allocate instance");
        return NULL;
    }

    if (module_dir) {
        strncpy(inst->module_dir, module_dir, sizeof(inst->module_dir) - 1);
    }

    /* Set default parameters */
    inst->param_input = 0.7f;
    inst->param_drive = 0.0f;
    inst->param_color = 0.0f;
    inst->param_wobble = 0.0f;
    inst->param_tone = 0.5f;
    inst->param_output = 1.0f;
    inst->randState = 0x1234567u;

    /* Initialize DSP modules */
    v2_GS_Init(inst, SAMPLE_RATE);
    v2_GS_SetParams(inst, inst->param_drive, inst->param_color);
    v2_GS_SetOutputLevel(inst, inst->param_output);

    v2_TapeSat_SetDrive(inst, inst->param_color);

    v2_WF_Init(inst, SAMPLE_RATE);
    v2_WF_SetAmount(inst, inst->param_wobble);

    v2_Tone_Init(inst);
    v2_Tone_SetAmount(inst, inst->param_tone);

    v2_log("Instance created");
    return inst;
}

static void v2_destroy_instance(void *instance) {
    tapescam_instance_t *inst = (tapescam_instance_t*)instance;
    if (!inst) return;

    v2_log("Destroying instance");

    if (inst->delayBufL) free(inst->delayBufL);
    if (inst->delayBufR) free(inst->delayBufR);

    free(inst);
}

static void v2_process_block(void *instance, int16_t *audio_inout, int frames) {
    tapescam_instance_t *inst = (tapescam_instance_t*)instance;
    if (!inst) return;

    float left_buf[512];
    float right_buf[512];

    int remaining = frames;
    int offset = 0;

    while (remaining > 0) {
        int chunk = remaining > 512 ? 512 : remaining;

        /* Convert to float and apply input level */
        float inputGain = inst->param_input;
        for (int i = 0; i < chunk; i++) {
            left_buf[i] = (audio_inout[(offset + i) * 2] / 32768.0f) * inputGain;
            right_buf[i] = (audio_inout[(offset + i) * 2 + 1] / 32768.0f) * inputGain;
        }

        /* Update controls */
        v2_TapeSat_UpdateControls(inst);
        v2_WF_UpdateControls(inst);
        v2_Tone_UpdateControls(inst, SAMPLE_RATE);

        /* Process chain: GainStage -> TapeSat -> WowFlutter -> Tone */
        v2_GS_Process(inst, left_buf, right_buf, chunk);
        v2_TapeSat_Process(inst, left_buf, right_buf, chunk);
        v2_WF_Process(inst, left_buf, right_buf, left_buf, right_buf, chunk, SAMPLE_RATE);
        v2_Tone_Process(inst, left_buf, right_buf, chunk);

        /* Final soft clip and convert back */
        for (int i = 0; i < chunk; i++) {
            float out_l = tanhf(left_buf[i] * 0.95f);
            float out_r = tanhf(right_buf[i] * 0.95f);
            audio_inout[(offset + i) * 2] = (int16_t)(Clamp(out_l, -1.0f, 1.0f) * 32767.0f);
            audio_inout[(offset + i) * 2 + 1] = (int16_t)(Clamp(out_r, -1.0f, 1.0f) * 32767.0f);
        }

        remaining -= chunk;
        offset += chunk;
    }
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    tapescam_instance_t *inst = (tapescam_instance_t*)instance;
    if (!inst) return;

    float v = atof(val);
    v = Clamp(v, 0.0f, 1.0f);

    if (strcmp(key, "input") == 0) {
        inst->param_input = v;
    } else if (strcmp(key, "drive") == 0) {
        inst->param_drive = v;
        v2_GS_SetParams(inst, inst->param_drive, inst->param_color);
    } else if (strcmp(key, "color") == 0) {
        inst->param_color = v;
        v2_GS_SetParams(inst, inst->param_drive, inst->param_color);
        v2_TapeSat_SetDrive(inst, v);
    } else if (strcmp(key, "wobble") == 0) {
        inst->param_wobble = v;
        v2_WF_SetAmount(inst, v);
    } else if (strcmp(key, "tone") == 0) {
        inst->param_tone = v;
        v2_Tone_SetAmount(inst, v);
    } else if (strcmp(key, "output") == 0) {
        inst->param_output = v;
        v2_GS_SetOutputLevel(inst, v);
    }
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    tapescam_instance_t *inst = (tapescam_instance_t*)instance;
    if (!inst) return -1;

    if (strcmp(key, "input") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_input);
    if (strcmp(key, "drive") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_drive);
    if (strcmp(key, "color") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_color);
    if (strcmp(key, "wobble") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_wobble);
    if (strcmp(key, "tone") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_tone);
    if (strcmp(key, "output") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_output);
    if (strcmp(key, "name") == 0) return snprintf(buf, buf_len, "TAPESCAM");

    /* UI hierarchy for shadow parameter editor */
    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *hierarchy = "{"
            "\"modes\":null,"
            "\"levels\":{"
                "\"root\":{"
                    "\"children\":null,"
                    "\"knobs\":[\"input\",\"drive\",\"color\",\"wobble\",\"tone\",\"output\"],"
                    "\"params\":[\"input\",\"drive\",\"color\",\"wobble\",\"tone\",\"output\"]"
                "}"
            "}"
        "}";
        int len = strlen(hierarchy);
        if (len < buf_len) {
            strcpy(buf, hierarchy);
            return len;
        }
        return -1;
    }

    /* Chain params metadata for shadow parameter editor */
    if (strcmp(key, "chain_params") == 0) {
        const char *params_json = "["
            "{\"key\":\"input\",\"name\":\"Input\",\"type\":\"float\",\"min\":0,\"max\":1},"
            "{\"key\":\"drive\",\"name\":\"Drive\",\"type\":\"float\",\"min\":0,\"max\":1},"
            "{\"key\":\"color\",\"name\":\"Color\",\"type\":\"float\",\"min\":0,\"max\":1},"
            "{\"key\":\"wobble\",\"name\":\"Wobble\",\"type\":\"float\",\"min\":0,\"max\":1},"
            "{\"key\":\"tone\",\"name\":\"Tone\",\"type\":\"float\",\"min\":0,\"max\":1},"
            "{\"key\":\"output\",\"name\":\"Output\",\"type\":\"float\",\"min\":0,\"max\":1}"
        "]";
        int len = strlen(params_json);
        if (len < buf_len) {
            strcpy(buf, params_json);
            return len;
        }
        return -1;
    }

    return -1;
}

static audio_fx_api_v2_t g_fx_api_v2;

audio_fx_api_v2_t* move_audio_fx_init_v2(const host_api_v1_t *host) {
    g_host = host;

    memset(&g_fx_api_v2, 0, sizeof(g_fx_api_v2));
    g_fx_api_v2.api_version = AUDIO_FX_API_VERSION_2;
    g_fx_api_v2.create_instance = v2_create_instance;
    g_fx_api_v2.destroy_instance = v2_destroy_instance;
    g_fx_api_v2.process_block = v2_process_block;
    g_fx_api_v2.set_param = v2_set_param;
    g_fx_api_v2.get_param = v2_get_param;

    v2_log("TAPESCAM v2 plugin initialized");

    return &g_fx_api_v2;
}
