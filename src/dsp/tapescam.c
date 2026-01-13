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

/* GainStage parameters */
static float g_gs_trimGainLin = 1.0f;
static float g_gs_channelGainLin = 1.0f;
static float g_gs_masterVolLin = 1.0f;
static float g_gs_driveNorm = 0.0f;
static float g_gs_character = 0.0f;
static float g_gs_characterDriveScale = 1.0f;
static float g_gs_stage1Softness = 1.0f;
static float g_gs_stage2Softness = 1.2f;
static float g_gs_stage2Asymmetry = 0.05f;

/* Biquad filter state for pre/post filtering */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} GS_Biquad;

static GS_Biquad g_gs_preHP[2];
static GS_Biquad g_gs_preLP[2];
static GS_Biquad g_gs_postLP[2];

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

static void GS_Init(float sampleRate) {
    g_gs_trimGainLin = 1.0f;
    g_gs_channelGainLin = 1.0f;
    g_gs_masterVolLin = 1.0f;
    g_gs_driveNorm = 0.0f;
    g_gs_character = 0.0f;
    g_gs_characterDriveScale = 1.0f;
    g_gs_stage1Softness = 1.0f;
    g_gs_stage2Softness = 1.2f;
    g_gs_stage2Asymmetry = 0.05f;

    /* Initialize filters - from GainStageModule constants */
    float preHpCutHz = 160.0f * 0.9f;   /* kMkPreHighpassHz */
    float preLpCutHz = 5200.0f * 1.1f;  /* kMkPreLowpassHz */
    float postLpCutHz = 17500.0f;

    for (int ch = 0; ch < 2; ch++) {
        memset(&g_gs_preHP[ch], 0, sizeof(GS_Biquad));
        memset(&g_gs_preLP[ch], 0, sizeof(GS_Biquad));
        memset(&g_gs_postLP[ch], 0, sizeof(GS_Biquad));
        g_gs_preHP[ch].b0 = 1.0f;
        g_gs_preLP[ch].b0 = 1.0f;
        g_gs_postLP[ch].b0 = 1.0f;

        GS_BiquadSetHighpass(&g_gs_preHP[ch], sampleRate, preHpCutHz, 0.7071f);
        GS_BiquadSetLowpass(&g_gs_preLP[ch], sampleRate, preLpCutHz, 0.7071f);
        GS_BiquadSetLowpass(&g_gs_postLP[ch], sampleRate, postLpCutHz, 0.7071f);
    }
}

/* Set drive and color parameters - matches VST updateModulesFromParameters */
static void GS_SetParams(float drive, float color) {
    float driveNorm = Clamp(drive, 0.0f, 1.0f);
    float driveShaped = driveNorm * driveNorm;
    float colorNorm = Clamp(color, 0.0f, 1.0f);

    /* From VST: trimDb and channelDb calculations */
    float trimDb = -2.0f + driveShaped * 34.0f;      /* -2 to +32 dB */
    float channelDb = -2.0f + driveNorm * 26.0f;    /* -2 to +24 dB */

    g_gs_trimGainLin = dBToLin(trimDb);
    g_gs_channelGainLin = dBToLin(channelDb);
    g_gs_driveNorm = driveNorm;
    g_gs_character = colorNorm;

    /* Character affects clipping behavior */
    g_gs_characterDriveScale = 0.8f + 1.2f * colorNorm;
    g_gs_stage1Softness = 0.8f + 0.4f * (1.0f - colorNorm);
    g_gs_stage2Softness = 1.05f + 1.1f * colorNorm;
    g_gs_stage2Asymmetry = 0.02f + 0.08f * colorNorm;
}

static void GS_SetOutputLevel(float level) {
    float levelDb = -12.0f + level * 18.0f;  /* -12 to +6 dB */
    g_gs_masterVolLin = dBToLin(levelDb);
}

/* Core clipping function - from ApplyClippingCore */
static float GS_ApplyClipping(float x, float softness) {
    float drive = 1.0f + g_gs_driveNorm * g_gs_characterDriveScale;
    float v = x * drive;

    /* Soft tanh clipping (default mode) */
    v = tanhf(v * softness);

    /* Add asymmetry for even harmonics */
    float asymBase = 0.01f + 0.03f * g_gs_character;
    v += asymBase * v * v * (v >= 0.0f ? 1.0f : -1.0f);

    return v;
}

/* Stage 1 clipping */
static float GS_ApplyStage1(float x) {
    return GS_ApplyClipping(x, g_gs_stage1Softness);
}

/* Stage 2 clipping with channel-dependent asymmetry */
static float GS_ApplyStage2(float x, int channel) {
    float v = GS_ApplyClipping(x, g_gs_stage2Softness);
    float asym = g_gs_stage2Asymmetry * (channel == 0 ? 1.0f : -1.0f);
    v += asym * v * v;
    return v;
}

/* Main GainStage process - simplified from GainStageModule::Process */
static void GS_Process(float *left, float *right, int size) {
    for (int i = 0; i < size; i++) {
        /* Stage 0: Input */
        float stage0L = left[i];
        float stage0R = right[i];

        /* Stage 1: First gain + clipping */
        float stage1InL = stage0L * g_gs_trimGainLin;
        float stage1InR = stage0R * g_gs_trimGainLin;
        float stage1OutL = GS_ApplyStage1(stage1InL);
        float stage1OutR = GS_ApplyStage1(stage1InR);

        /* Pre-EQ filtering */
        float preEqL = GS_BiquadProcess(&g_gs_preLP[0], GS_BiquadProcess(&g_gs_preHP[0], stage1OutL));
        float preEqR = GS_BiquadProcess(&g_gs_preLP[1], GS_BiquadProcess(&g_gs_preHP[1], stage1OutR));

        /* Stage 2: Second gain + clipping */
        float stage2InL = preEqL * g_gs_channelGainLin;
        float stage2InR = preEqR * g_gs_channelGainLin;
        float stage2OutL = GS_ApplyStage2(stage2InL, 0);
        float stage2OutR = GS_ApplyStage2(stage2InR, 1);

        /* Post filtering */
        float postL = GS_BiquadProcess(&g_gs_postLP[0], stage2OutL);
        float postR = GS_BiquadProcess(&g_gs_postLP[1], stage2OutR);

        /* Output with master volume */
        left[i] = Clamp(postL * g_gs_masterVolLin, -0.98f, 0.98f);
        right[i] = Clamp(postR * g_gs_masterVolLin, -0.98f, 0.98f);
    }
}

/* ============================================================================
 * TAPE SATURATION MODULE - Port from TapeSatModule
 * ============================================================================ */

static float g_sat_targetDrive = 0.0f;
static float g_sat_smoothedDrive = 0.0f;
static float g_sat_factor = 0.0f;
static const float g_sat_smoothCoeff = 0.3f;

static void TapeSat_SetDrive(float normalizedDrive) {
    float shaped = Clamp(normalizedDrive, 0.0f, 1.0f);
    shaped = powf(shaped, 0.8f);
    g_sat_targetDrive = shaped;
}

static void TapeSat_UpdateControls(void) {
    float delta = g_sat_targetDrive - g_sat_smoothedDrive;
    g_sat_smoothedDrive += delta * g_sat_smoothCoeff;
    if (fabsf(g_sat_targetDrive - g_sat_smoothedDrive) < 1e-4f) {
        g_sat_smoothedDrive = g_sat_targetDrive;
    }
    g_sat_factor = Clamp(g_sat_smoothedDrive * 1.35f, 0.0f, 2.0f);
}

static void TapeSat_Process(float *left, float *right, int size) {
    if (g_sat_smoothedDrive < 0.01f) return;
    float satAmount = g_sat_factor;
    if (satAmount < 0.01f) return;

    float drive = 1.0f + satAmount * 2.5f;
    for (int i = 0; i < size; i++) {
        left[i] = tanhf(left[i] * drive);
        right[i] = tanhf(right[i] * drive);
    }
}

/* ============================================================================
 * WOW/FLUTTER MODULE - Port from WowFlutterModule
 * ============================================================================ */

#define kMaxDelaySamples 2048
#define kBaseDelayMsL 5.0f
#define kBaseDelayMsR 5.5f

static float g_delayBufL[kMaxDelaySamples];
static float g_delayBufR[kMaxDelaySamples];
static size_t g_writeIndex = 0;

static float g_wf_targetAmount = 0.0f;
static float g_wf_smoothedAmount = 0.0f;
static float g_wf_activationRamp = 0.0f;
static float g_wf_crossfadeGain = 0.0f;
static float g_wf_depthShape = 0.0f;
static float g_wf_slewAlpha = 0.9f;

static float g_wf_wowAmount = 0.0f;
static float g_wf_flutterAmount = 0.0f;
static float g_wf_wowRateHz = 0.3f;
static float g_wf_flutterRateHz = 4.0f;

static float g_wf_wowPhaseL = 0.0f;
static float g_wf_wowPhaseR = 0.25f;
static float g_wf_fltPhaseL = 0.0f;
static float g_wf_fltPhaseR = 0.33f;

static float g_wf_readStateL = 0.0f;
static float g_wf_readStateR = 0.0f;
static float g_wf_prevReadPosL = 0.0f;
static float g_wf_prevReadPosR = 0.0f;
static float g_wf_baseDelaySamplesL = 0.0f;
static float g_wf_baseDelaySamplesR = 0.0f;

static float g_wf_wowNoiseStateL = 0.0f;
static float g_wf_wowNoiseStateR = 0.0f;
static float g_wf_flutterNoiseStateL = 0.0f;
static float g_wf_flutterNoiseStateR = 0.0f;

static uint32_t g_randState = 0x1234567u;

static float WF_NextRand(void) {
    g_randState = g_randState * 1664525u + 1013904223u;
    return (float)g_randState * 2.3283064365386963e-10f;
}

static float WF_NextRandCentered(void) {
    return WF_NextRand() * 2.0f - 1.0f;
}

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

static void WF_Init(float sampleRate) {
    memset(g_delayBufL, 0, sizeof(g_delayBufL));
    memset(g_delayBufR, 0, sizeof(g_delayBufR));
    g_writeIndex = 0;

    g_wf_baseDelaySamplesL = fminf(sampleRate * (kBaseDelayMsL * 0.001f), (float)kMaxDelaySamples - 4.0f);
    g_wf_baseDelaySamplesR = fminf(sampleRate * (kBaseDelayMsR * 0.001f), (float)kMaxDelaySamples - 4.0f);
    g_wf_readStateL = g_wf_baseDelaySamplesL;
    g_wf_readStateR = g_wf_baseDelaySamplesR;

    g_randState ^= (uint32_t)sampleRate;
}

static void WF_SetAmount(float amount) {
    g_wf_targetAmount = Clamp(amount, 0.0f, 1.0f);
}

static void WF_UpdateControls(void) {
    g_wf_smoothedAmount += (g_wf_targetAmount - g_wf_smoothedAmount) * 0.3f;
    float targetActive = (g_wf_targetAmount > 0.0005f) ? 1.0f : 0.0f;
    g_wf_activationRamp += (targetActive - g_wf_activationRamp) * 0.4f;

    float amt = g_wf_smoothedAmount * g_wf_activationRamp;
    g_wf_depthShape = amt * amt;

    float depthBlendWow = 0.05f + 0.95f * g_wf_depthShape;
    float depthBlendFlutter = 0.03f + 0.97f * g_wf_depthShape;

    float targetSlew = (amt <= 0.0005f) ? 0.97f : Clamp(0.94f - 0.28f * g_wf_depthShape, 0.7f, 0.95f);
    g_wf_slewAlpha += (targetSlew - g_wf_slewAlpha) * 0.2f;

    g_wf_wowAmount = depthBlendWow * g_wf_activationRamp;
    g_wf_flutterAmount = depthBlendFlutter * g_wf_activationRamp;
}

static void WF_Process(float *inL, float *inR, float *outL, float *outR, int size, float sampleRate) {
    float wowAmt = g_wf_wowAmount;
    float flutterAmt = g_wf_flutterAmount;
    float invSr = 1.0f / sampleRate;

    int bypassDelay = (g_wf_activationRamp < 1.0e-4f && wowAmt <= 1.0e-5f && flutterAmt <= 1.0e-5f);
    if (bypassDelay) {
        for (int i = 0; i < size; i++) {
            outL[i] = inL[i];
            outR[i] = inR[i];
            g_delayBufL[g_writeIndex] = inL[i];
            g_delayBufR[g_writeIndex] = inR[i];
            g_writeIndex++;
            if (g_writeIndex >= kMaxDelaySamples) g_writeIndex = 0;
        }
        return;
    }

    float crossfadeAlpha = 0.004f;
    float crossfadeGain = g_wf_crossfadeGain;

    float maxWowDepthSec = 0.035f;
    float maxFlutterDepthSec = 0.002f;
    float maxDevSec = 0.0042f;

    for (int i = 0; i < size; i++) {
        float xL = inL[i], xR = inR[i];
        crossfadeGain += (g_wf_activationRamp - crossfadeGain) * crossfadeAlpha;

        g_wf_wowNoiseStateL += (WF_NextRandCentered() - g_wf_wowNoiseStateL) * 0.00018f;
        g_wf_wowNoiseStateR += (WF_NextRandCentered() - g_wf_wowNoiseStateR) * 0.00018f;
        g_wf_flutterNoiseStateL += (WF_NextRandCentered() - g_wf_flutterNoiseStateL) * 0.00025f;
        g_wf_flutterNoiseStateR += (WF_NextRandCentered() - g_wf_flutterNoiseStateR) * 0.00025f;

        g_wf_wowPhaseL += g_wf_wowRateHz * invSr;
        if (g_wf_wowPhaseL >= 1.0f) g_wf_wowPhaseL -= 1.0f;
        g_wf_fltPhaseL += g_wf_flutterRateHz * invSr;
        if (g_wf_fltPhaseL >= 1.0f) g_wf_fltPhaseL -= 1.0f;

        g_wf_wowPhaseR += g_wf_wowRateHz * 1.03f * invSr;
        if (g_wf_wowPhaseR >= 1.0f) g_wf_wowPhaseR -= 1.0f;
        g_wf_fltPhaseR += g_wf_flutterRateHz * 0.97f * invSr;
        if (g_wf_fltPhaseR >= 1.0f) g_wf_fltPhaseR -= 1.0f;

        float depthSlow = sqrtf(fmaxf(g_wf_depthShape, 0.0f));
        float wowNoiseMix = Clamp(0.4f * (0.10f + 0.45f * depthSlow), 0.0f, 0.65f);
        float flutterNoiseMix = Clamp(0.2f * (0.12f + 0.45f * depthSlow), 0.0f, 0.7f);

        float wowShapeL = (1.0f - wowNoiseMix) * sinf(kTwoPi * g_wf_wowPhaseL) + wowNoiseMix * g_wf_wowNoiseStateL;
        float flutterShapeL = (1.0f - flutterNoiseMix) * sinf(kTwoPi * g_wf_fltPhaseL) + flutterNoiseMix * g_wf_flutterNoiseStateL;
        float wowShapeR = (1.0f - wowNoiseMix) * sinf(kTwoPi * g_wf_wowPhaseR) + wowNoiseMix * g_wf_wowNoiseStateR;
        float flutterShapeR = (1.0f - flutterNoiseMix) * sinf(kTwoPi * g_wf_fltPhaseR) + flutterNoiseMix * g_wf_flutterNoiseStateR;

        float devSamplesL = (maxWowDepthSec * wowAmt * wowShapeL + maxFlutterDepthSec * flutterAmt * flutterShapeL) * sampleRate;
        float devSamplesR = (maxWowDepthSec * wowAmt * 1.03f * wowShapeR + maxFlutterDepthSec * flutterAmt * 0.97f * flutterShapeR) * sampleRate;

        float maxDevSamples = maxDevSec * sampleRate;
        devSamplesL = Clamp(devSamplesL, -maxDevSamples, maxDevSamples);
        devSamplesR = Clamp(devSamplesR, -maxDevSamples, maxDevSamples);

        float targetSamplesL = fmaxf(g_wf_baseDelaySamplesL + devSamplesL, 0.0002f * sampleRate);
        float targetSamplesR = fmaxf(g_wf_baseDelaySamplesR + devSamplesR, 0.0002f * sampleRate);

        g_wf_readStateL = g_wf_slewAlpha * g_wf_readStateL + (1.0f - g_wf_slewAlpha) * targetSamplesL;
        g_wf_readStateR = g_wf_slewAlpha * g_wf_readStateR + (1.0f - g_wf_slewAlpha) * targetSamplesR;

        float readIndexL = (float)g_writeIndex - g_wf_readStateL;
        float readIndexR = (float)g_writeIndex - g_wf_readStateR;
        if (readIndexL < 0.0f) readIndexL += (float)kMaxDelaySamples;
        if (readIndexR < 0.0f) readIndexR += (float)kMaxDelaySamples;

        float delayedL = WF_InterpolateCubic(g_delayBufL, kMaxDelaySamples, readIndexL);
        float delayedR = WF_InterpolateCubic(g_delayBufR, kMaxDelaySamples, readIndexR);

        g_delayBufL[g_writeIndex] = xL;
        g_delayBufR[g_writeIndex] = xR;
        g_writeIndex++;
        if (g_writeIndex >= kMaxDelaySamples) g_writeIndex = 0;

        float wet = crossfadeGain, dry = 1.0f - wet;
        outL[i] = dry * xL + wet * delayedL;
        outR[i] = dry * xR + wet * delayedR;
    }
    g_wf_crossfadeGain = crossfadeGain;
}

/* ============================================================================
 * TONE MODULE - Port from ToneModule (Tilt EQ)
 * ============================================================================ */

typedef struct {
    float b0, b1, b2, a1, a2, z1, z2;
} ToneBiquad;

static ToneBiquad g_tone_lowShelfL, g_tone_lowShelfR;
static ToneBiquad g_tone_highShelfL, g_tone_highShelfR;
static float g_tone_targetAmount = 0.5f;
static float g_tone_smoothedAmount = 0.5f;
static float g_tone_bassGainDb = 0.0f;
static float g_tone_trebleGainDb = 0.0f;

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

static void Tone_Init(float sampleRate) {
    memset(&g_tone_lowShelfL, 0, sizeof(ToneBiquad));
    memset(&g_tone_lowShelfR, 0, sizeof(ToneBiquad));
    memset(&g_tone_highShelfL, 0, sizeof(ToneBiquad));
    memset(&g_tone_highShelfR, 0, sizeof(ToneBiquad));
    g_tone_lowShelfL.b0 = g_tone_lowShelfR.b0 = 1.0f;
    g_tone_highShelfL.b0 = g_tone_highShelfR.b0 = 1.0f;
    g_tone_targetAmount = 0.5f;
    g_tone_smoothedAmount = 0.5f;
}

static void Tone_SetAmount(float amount) {
    g_tone_targetAmount = Clamp(amount, 0.0f, 1.0f);
}

static void Tone_UpdateControls(float sampleRate) {
    g_tone_smoothedAmount += (g_tone_targetAmount - g_tone_smoothedAmount) * 0.5f;
    float normalized = g_tone_smoothedAmount * 2.0f - 1.0f;
    float shaped = normalized * (1.0f + 0.9f * fabsf(normalized));
    g_tone_bassGainDb = -shaped * 15.0f;
    g_tone_trebleGainDb = shaped * 15.0f;

    Tone_ComputeShelf(&g_tone_lowShelfL, g_tone_bassGainDb, 120.0f, 0, sampleRate);
    Tone_ComputeShelf(&g_tone_lowShelfR, g_tone_bassGainDb, 120.0f, 0, sampleRate);
    Tone_ComputeShelf(&g_tone_highShelfL, g_tone_trebleGainDb, 6000.0f, 1, sampleRate);
    Tone_ComputeShelf(&g_tone_highShelfR, g_tone_trebleGainDb, 6000.0f, 1, sampleRate);
}

static inline float Tone_ProcessBiquad(ToneBiquad *bq, float x) {
    float y = x * bq->b0 + bq->z1;
    bq->z1 = x * bq->b1 + bq->z2 - bq->a1 * y;
    bq->z2 = x * bq->b2 - bq->a2 * y;
    return y;
}

static void Tone_Process(float *bufL, float *bufR, int size) {
    for (int i = 0; i < size; i++) {
        float y = Tone_ProcessBiquad(&g_tone_lowShelfL, bufL[i]);
        bufL[i] = Tone_ProcessBiquad(&g_tone_highShelfL, y);
        y = Tone_ProcessBiquad(&g_tone_lowShelfR, bufR[i]);
        bufR[i] = Tone_ProcessBiquad(&g_tone_highShelfR, y);
    }
}

/* ============================================================================
 * PLUGIN STATE AND API
 * ============================================================================ */

static const host_api_v1_t *g_host = NULL;
static audio_fx_api_v1_t g_fx_api;

/* 6 VST Knobs mapped to Move knobs 2-7 (0.0 to 1.0) */
static float g_param_input = 0.7f;    /* Knob 2: INPUT level (default 0.7) */
static float g_param_drive = 0.0f;    /* Knob 3: DRIVE (default 0) */
static float g_param_color = 0.0f;    /* Knob 4: COLOR/saturation (default 0) */
static float g_param_wobble = 0.0f;   /* Knob 5: WOW/FLUTTER (default 0) */
static float g_param_tone = 0.5f;     /* Knob 6: TONE (default 0.5) */
static float g_param_output = 1.0f;   /* Knob 7: OUTPUT level (default 1.0) */

static void fx_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[tapescam] %s", msg);
        g_host->log(buf);
    }
}

static int fx_on_load(const char *module_dir, const char *config_json) {
    fx_log("TAPESCAM loading (6-knob VST port)");

    GS_Init(SAMPLE_RATE);
    GS_SetParams(g_param_drive, g_param_color);
    GS_SetOutputLevel(g_param_output);

    TapeSat_SetDrive(g_param_color);

    WF_Init(SAMPLE_RATE);
    WF_SetAmount(g_param_wobble);

    Tone_Init(SAMPLE_RATE);
    Tone_SetAmount(g_param_tone);

    fx_log("TAPESCAM initialized");
    return 0;
}

static void fx_on_unload(void) {
    fx_log("TAPESCAM unloading");
}

static void fx_process_block(int16_t *audio_inout, int frames) {
    float left_buf[512];
    float right_buf[512];

    int remaining = frames;
    int offset = 0;

    while (remaining > 0) {
        int chunk = remaining > 512 ? 512 : remaining;

        /* Convert to float and apply input level */
        float inputGain = g_param_input;
        for (int i = 0; i < chunk; i++) {
            left_buf[i] = (audio_inout[(offset + i) * 2] / 32768.0f) * inputGain;
            right_buf[i] = (audio_inout[(offset + i) * 2 + 1] / 32768.0f) * inputGain;
        }

        /* Update controls */
        TapeSat_UpdateControls();
        WF_UpdateControls();
        Tone_UpdateControls(SAMPLE_RATE);

        /* Process chain: GainStage -> TapeSat -> WowFlutter -> Tone */
        GS_Process(left_buf, right_buf, chunk);
        TapeSat_Process(left_buf, right_buf, chunk);
        WF_Process(left_buf, right_buf, left_buf, right_buf, chunk, SAMPLE_RATE);
        Tone_Process(left_buf, right_buf, chunk);

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

static void fx_set_param(const char *key, const char *val) {
    float v = atof(val);
    v = Clamp(v, 0.0f, 1.0f);

    if (strcmp(key, "input") == 0) {
        g_param_input = v;
    } else if (strcmp(key, "drive") == 0) {
        g_param_drive = v;
        GS_SetParams(g_param_drive, g_param_color);
    } else if (strcmp(key, "color") == 0) {
        g_param_color = v;
        GS_SetParams(g_param_drive, g_param_color);
        TapeSat_SetDrive(v);
    } else if (strcmp(key, "wobble") == 0) {
        g_param_wobble = v;
        WF_SetAmount(v);
    } else if (strcmp(key, "tone") == 0) {
        g_param_tone = v;
        Tone_SetAmount(v);
    } else if (strcmp(key, "output") == 0) {
        g_param_output = v;
        GS_SetOutputLevel(v);
    }
}

static int fx_get_param(const char *key, char *buf, int buf_len) {
    if (strcmp(key, "input") == 0) return snprintf(buf, buf_len, "%.2f", g_param_input);
    if (strcmp(key, "drive") == 0) return snprintf(buf, buf_len, "%.2f", g_param_drive);
    if (strcmp(key, "color") == 0) return snprintf(buf, buf_len, "%.2f", g_param_color);
    if (strcmp(key, "wobble") == 0) return snprintf(buf, buf_len, "%.2f", g_param_wobble);
    if (strcmp(key, "tone") == 0) return snprintf(buf, buf_len, "%.2f", g_param_tone);
    if (strcmp(key, "output") == 0) return snprintf(buf, buf_len, "%.2f", g_param_output);
    if (strcmp(key, "name") == 0) return snprintf(buf, buf_len, "TAPESCAM");
    return -1;
}

audio_fx_api_v1_t* move_audio_fx_init_v1(const host_api_v1_t *host) {
    g_host = host;

    memset(&g_fx_api, 0, sizeof(g_fx_api));
    g_fx_api.api_version = AUDIO_FX_API_VERSION;
    g_fx_api.on_load = fx_on_load;
    g_fx_api.on_unload = fx_on_unload;
    g_fx_api.process_block = fx_process_block;
    g_fx_api.set_param = fx_set_param;
    g_fx_api.get_param = fx_get_param;

    return &g_fx_api;
}
