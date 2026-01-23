# TAPESCAM DSP Alignment with TapeScamVST

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Fix parameter scaling and add missing DSP modules to match TapeScamVST behavior

**Architecture:** The current tapescam.c applies output level INSIDE GainStage (before TapeSat), causing loudness that can't be attenuated. We need to restructure to apply output level AFTER all processing. Additionally, we need to add missing parameters (noise, age, speed, compression, widen) and their corresponding DSP modules (HissDrop, LoFiCompressor, Dropout).

**Tech Stack:** C, Move Anything Audio FX API v2

---

## Critical Issues to Fix

1. **Output level applied in wrong place** - Currently inside GainStage, should be after all processing
2. **Missing parameters** - noise, age (0-2), speed (0-2), compression (0-2), widen (bool)
3. **Missing DSP modules** - HissDropModule, LoFiCompressor, DropoutModule
4. **Missing tape age/speed modifiers** - headroom adjustment, saturation scaling, wow/flutter scaling

---

### Task 1: Fix Output Level Application Order

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Change GainStage to use unity masterVol internally**

In `v2_GS_SetOutputLevel`, change to set masterVol to 0dB (unity) internally, and store the output level for post-chain application:

```c
static void v2_GS_SetOutputLevel(tapescam_instance_t *inst, float level) {
    /* Store for post-chain application - range: -12dB to +6dB */
    float levelDb = -12.0f + level * 18.0f;
    inst->param_outputLevelLin = dBToLin(levelDb);
    /* GainStage now runs at unity */
    inst->gs_masterVolLin = 1.0f;
}
```

**Step 2: Add param_outputLevelLin field to instance struct**

Add after `param_output`:
```c
float param_outputLevelLin;  /* Computed linear output level */
```

**Step 3: Apply output level after all processing in v2_process_block**

Replace the final soft clip loop with:
```c
/* Apply post-chain output level with smoothing, then soft clip */
for (int i = 0; i < chunk; i++) {
    float out_l = left_buf[i] * inst->param_outputLevelLin;
    float out_r = right_buf[i] * inst->param_outputLevelLin;
    out_l = tanhf(out_l * 0.95f);
    out_r = tanhf(out_r * 0.95f);
    audio_inout[(offset + i) * 2] = (int16_t)(Clamp(out_l, -1.0f, 1.0f) * 32767.0f);
    audio_inout[(offset + i) * 2 + 1] = (int16_t)(Clamp(out_r, -1.0f, 1.0f) * 32767.0f);
}
```

**Step 4: Update v2_create_instance to initialize new field**

Add after existing param initialization:
```c
inst->param_outputLevelLin = 1.0f;  /* Unity default */
```

**Step 5: Build and verify**

Run: `./scripts/build.sh`
Expected: Clean build with no errors

**Step 6: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "fix: apply output level after all processing to fix loudness scaling"
```

---

### Task 2: Add Missing Parameters to Instance Struct

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Add new parameter fields to tapescam_instance_t**

Add after `param_output`:
```c
float param_outputLevelLin;  /* Computed linear output level */
float param_noise;           /* 0-1: tape hiss amount */
int param_age;               /* 0=NEW, 1=USED, 2=WORN */
int param_speed;             /* 0=HIGH, 1=STD, 2=LOW */
int param_compression;       /* 0=OFF, 1=LITE, 2=HEAVY */
int param_widen;             /* 0=OFF, 1=ON */
```

**Step 2: Initialize in v2_create_instance**

Add after existing param initialization:
```c
inst->param_noise = 0.0f;
inst->param_age = 0;      /* NEW */
inst->param_speed = 0;    /* HIGH (cleanest) */
inst->param_compression = 0;  /* OFF */
inst->param_widen = 1;    /* ON by default */
```

**Step 3: Build and verify**

Run: `./scripts/build.sh`
Expected: Clean build

**Step 4: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: add missing parameter fields (noise, age, speed, compression, widen)"
```

---

### Task 3: Implement HissDrop Module (Tape Noise)

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Add HissDrop state to instance struct**

Add after tone state:
```c
/* HissDrop state */
float hiss_targetAmount;
float hiss_smoothedAmount;
float hiss_levelLin;
float hiss_noiseColorFactor;
float hiss_pinkState[2][7];  /* 7 pink noise state vars per channel */
```

**Step 2: Add HissDrop initialization function**

```c
static void v2_Hiss_Init(tapescam_instance_t *inst) {
    inst->hiss_targetAmount = 0.0f;
    inst->hiss_smoothedAmount = 0.0f;
    inst->hiss_levelLin = 0.0f;
    inst->hiss_noiseColorFactor = 0.0f;
    memset(inst->hiss_pinkState, 0, sizeof(inst->hiss_pinkState));
}

static void v2_Hiss_SetAmount(tapescam_instance_t *inst, float amount) {
    inst->hiss_targetAmount = Clamp(amount, 0.0f, 1.0f);
}
```

**Step 3: Add HissDrop UpdateControls**

```c
static void v2_Hiss_UpdateControls(tapescam_instance_t *inst) {
    float delta = inst->hiss_targetAmount - inst->hiss_smoothedAmount;
    inst->hiss_smoothedAmount += delta * 0.5f;

    float amt = inst->hiss_smoothedAmount;
    if (amt < 0.0005f) {
        inst->hiss_levelLin = 0.0f;
        inst->hiss_noiseColorFactor = 0.0f;
        return;
    }

    /* Map 0-1 to -60dB to -6dB */
    float hissDb = -60.0f + amt * 54.0f;
    inst->hiss_levelLin = dBToLin(hissDb);
    inst->hiss_noiseColorFactor = amt;
}
```

**Step 4: Add pink noise processing**

```c
static float v2_Hiss_ProcessPink(tapescam_instance_t *inst, int ch, float white) {
    float *s = inst->hiss_pinkState[ch];
    s[0] = 0.99886f * s[0] + white * 0.0555179f;
    s[1] = 0.99332f * s[1] + white * 0.0750759f;
    s[2] = 0.96900f * s[2] + white * 0.1538520f;
    s[3] = 0.86650f * s[3] + white * 0.3104856f;
    s[4] = 0.55000f * s[4] + white * 0.5329522f;
    s[5] = -0.7616f * s[5] - white * 0.0168980f;
    float pink = s[0] + s[1] + s[2] + s[3] + s[4] + s[5] + s[6] + white * 0.5362f;
    s[6] = white * 0.115926f;
    return pink * 0.11f;
}
```

**Step 5: Add HissDrop Process function**

```c
static void v2_Hiss_Process(tapescam_instance_t *inst, float *left, float *right, int size) {
    if (inst->hiss_levelLin <= 0.0f) return;

    float colorFactor = inst->hiss_noiseColorFactor;
    float level = inst->hiss_levelLin;

    for (int i = 0; i < size; i++) {
        float whiteL = v2_WF_NextRandCentered(inst);
        float whiteR = v2_WF_NextRandCentered(inst);

        float noiseL = whiteL;
        float noiseR = whiteR;
        if (colorFactor >= 0.001f) {
            noiseL = (1.0f - colorFactor) * whiteL + colorFactor * v2_Hiss_ProcessPink(inst, 0, whiteL);
            noiseR = (1.0f - colorFactor) * whiteR + colorFactor * v2_Hiss_ProcessPink(inst, 1, whiteR);
        }

        left[i] += noiseL * level;
        right[i] += noiseR * level;
    }
}
```

**Step 6: Call from v2_create_instance and v2_process_block**

In v2_create_instance:
```c
v2_Hiss_Init(inst);
v2_Hiss_SetAmount(inst, inst->param_noise);
```

In v2_process_block, after Tone and before final output:
```c
v2_Hiss_UpdateControls(inst);
v2_Hiss_Process(inst, left_buf, right_buf, chunk);
```

**Step 7: Add noise parameter handling to v2_set_param**

```c
} else if (strcmp(key, "noise") == 0) {
    inst->param_noise = v;
    v2_Hiss_SetAmount(inst, v);
```

**Step 8: Add to v2_get_param**

```c
if (strcmp(key, "noise") == 0) return snprintf(buf, buf_len, "%.2f", inst->param_noise);
```

**Step 9: Build and verify**

Run: `./scripts/build.sh`
Expected: Clean build

**Step 10: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: add tape hiss/noise module"
```

---

### Task 4: Implement LoFi Compressor

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Add compressor state to instance struct**

```c
/* LoFi Compressor state */
int comp_mode;
float comp_envelopeL;
float comp_envelopeR;
float comp_smoothedGainL;
float comp_smoothedGainR;
float comp_attackCoeff;
float comp_releaseCoeff;
float comp_threshold;
float comp_ratio;
float comp_makeupGain;
float comp_maxBoost;
```

**Step 2: Add compressor functions**

```c
static void v2_Comp_Init(tapescam_instance_t *inst, float sampleRate) {
    inst->comp_mode = 0;
    inst->comp_envelopeL = 0.0f;
    inst->comp_envelopeR = 0.0f;
    inst->comp_smoothedGainL = 1.0f;
    inst->comp_smoothedGainR = 1.0f;
    v2_Comp_SetMode(inst, 0, sampleRate);
}

static void v2_Comp_SetMode(tapescam_instance_t *inst, int mode, float sampleRate) {
    inst->comp_mode = mode < 0 ? 0 : (mode > 2 ? 2 : mode);

    float attackMs = inst->comp_mode == 2 ? 2.0f : 5.0f;
    float releaseMs = inst->comp_mode == 2 ? 800.0f : 400.0f;

    inst->comp_attackCoeff = expf(-1.0f / (sampleRate * attackMs * 0.001f));
    inst->comp_releaseCoeff = expf(-1.0f / (sampleRate * releaseMs * 0.001f));

    switch (inst->comp_mode) {
        case 0:
            inst->comp_threshold = 1.0f;
            inst->comp_ratio = 1.0f;
            inst->comp_makeupGain = 1.0f;
            inst->comp_maxBoost = 1.0f;
            break;
        case 1:
            inst->comp_threshold = 0.45f;
            inst->comp_ratio = 8.0f;
            inst->comp_makeupGain = 0.7f;
            inst->comp_maxBoost = 6.0f;
            break;
        case 2:
            inst->comp_threshold = 0.65f;
            inst->comp_ratio = 18.0f;
            inst->comp_makeupGain = 0.5f;
            inst->comp_maxBoost = 12.0f;
            break;
    }
}

static void v2_Comp_Process(tapescam_instance_t *inst, float *left, float *right, int size) {
    if (inst->comp_mode == 0) return;

    for (int i = 0; i < size; i++) {
        float inL = left[i];
        float inR = right[i];
        float absL = fabsf(inL);
        float absR = fabsf(inR);

        /* Envelope follower */
        float envCoeffL = (absL > inst->comp_envelopeL) ? inst->comp_attackCoeff : inst->comp_releaseCoeff;
        float envCoeffR = (absR > inst->comp_envelopeR) ? inst->comp_attackCoeff : inst->comp_releaseCoeff;
        inst->comp_envelopeL = envCoeffL * inst->comp_envelopeL + (1.0f - envCoeffL) * absL;
        inst->comp_envelopeR = envCoeffR * inst->comp_envelopeR + (1.0f - envCoeffR) * absR;

        /* Calculate upward compression gain */
        float targetGainL = 1.0f, targetGainR = 1.0f;
        if (inst->comp_envelopeL < inst->comp_threshold) {
            float reduction = inst->comp_threshold - inst->comp_envelopeL;
            float normalizedReduction = reduction / inst->comp_threshold;
            targetGainL = (1.0f + (inst->comp_maxBoost - 1.0f) * normalizedReduction) * inst->comp_makeupGain;
        } else {
            targetGainL = inst->comp_makeupGain;
        }
        if (inst->comp_envelopeR < inst->comp_threshold) {
            float reduction = inst->comp_threshold - inst->comp_envelopeR;
            float normalizedReduction = reduction / inst->comp_threshold;
            targetGainR = (1.0f + (inst->comp_maxBoost - 1.0f) * normalizedReduction) * inst->comp_makeupGain;
        } else {
            targetGainR = inst->comp_makeupGain;
        }

        /* Smooth gain changes */
        float smoothCoeffL = (targetGainL < inst->comp_smoothedGainL) ? 0.2f : 0.000125f;
        float smoothCoeffR = (targetGainR < inst->comp_smoothedGainR) ? 0.2f : 0.000125f;
        inst->comp_smoothedGainL += (targetGainL - inst->comp_smoothedGainL) * smoothCoeffL;
        inst->comp_smoothedGainR += (targetGainR - inst->comp_smoothedGainR) * smoothCoeffR;

        /* Apply with safety limiting */
        float maxAllowedGain = inst->comp_mode == 2 ? 15.0f : 8.0f;
        float cappedGainL = fminf(inst->comp_smoothedGainL, maxAllowedGain);
        float cappedGainR = fminf(inst->comp_smoothedGainR, maxAllowedGain);

        float maxGainL = absL > 0.001f ? 0.9f / absL : cappedGainL;
        float maxGainR = absR > 0.001f ? 0.9f / absR : cappedGainR;

        left[i] = inL * fminf(cappedGainL, maxGainL);
        right[i] = inR * fminf(cappedGainR, maxGainR);
    }
}
```

**Step 3: Initialize and call in signal chain**

In v2_create_instance:
```c
v2_Comp_Init(inst, SAMPLE_RATE);
```

In v2_process_block, after Hiss and before final output:
```c
v2_Comp_Process(inst, left_buf, right_buf, chunk);
```

**Step 4: Add parameter handling**

In v2_set_param:
```c
} else if (strcmp(key, "compression") == 0) {
    inst->param_compression = (int)(v * 2.0f + 0.5f);  /* 0, 1, or 2 */
    v2_Comp_SetMode(inst, inst->param_compression, SAMPLE_RATE);
```

In v2_get_param:
```c
if (strcmp(key, "compression") == 0) return snprintf(buf, buf_len, "%d", inst->param_compression);
```

**Step 5: Build and verify**

Run: `./scripts/build.sh`

**Step 6: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: add LoFi compressor module (AGC pumping)"
```

---

### Task 5: Add Tape Age and Speed Modifiers

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Add headroom adjustment to GainStage**

Add field to instance:
```c
float gs_headroomAdjDb;
```

Add function:
```c
static void v2_GS_AdjustHeadroom(tapescam_instance_t *inst, float headroomDb) {
    inst->gs_headroomAdjDb = Clamp(headroomDb, -12.0f, 6.0f);
}
```

Modify v2_GS_SetParams to apply headroom:
```c
inst->gs_trimGainLin = dBToLin(trimDb + inst->gs_headroomAdjDb);
inst->gs_channelGainLin = dBToLin(channelDb + inst->gs_headroomAdjDb);
```

**Step 2: Add age/speed modifier application function**

```c
static void v2_ApplyAgeSpeedModifiers(tapescam_instance_t *inst) {
    int age = inst->param_age;
    int speed = inst->param_speed;

    /* Age: 0=NEW, 1=USED, 2=WORN */
    float ageNorm = Clamp(age / 2.0f, 0.0f, 1.0f);

    /* Headroom: worn tape has less headroom */
    float ageHeadroomDb = (ageNorm - 0.5f) * 1.5f;

    /* Speed: 0=HIGH (clean), 1=STD, 2=LOW (lo-fi) */
    float speedHeadroomDb = 0.0f;
    float speedSatMul = 1.0f;
    float wowDepthScale = 1.0f;
    float flutterDepthScale = 1.0f;

    switch (speed) {
        case 0:  /* HIGH */
            speedHeadroomDb = 2.0f;
            speedSatMul = 0.9f;
            wowDepthScale = 0.85f;
            flutterDepthScale = 0.85f;
            break;
        case 1:  /* STD */
            speedHeadroomDb = -0.5f;
            speedSatMul = 1.0f;
            wowDepthScale = 1.3f;
            flutterDepthScale = 1.0f;
            break;
        case 2:  /* LOW */
            speedHeadroomDb = -2.0f;
            speedSatMul = 1.25f;
            wowDepthScale = 1.8f;
            flutterDepthScale = 1.1f;
            break;
    }

    /* Age modifiers */
    float ageSatMul = 1.0f + 0.4f * ageNorm;
    float ageWowScale = 1.0f + 1.6f * ageNorm;
    float ageFlutterScale = 1.0f - 0.2f * ageNorm;

    /* Apply combined headroom */
    v2_GS_AdjustHeadroom(inst, ageHeadroomDb + speedHeadroomDb);

    /* Store combined modifiers for use by other modules */
    inst->mod_saturationMul = ageSatMul * speedSatMul;
    inst->mod_wowDepthScale = ageWowScale * wowDepthScale;
    inst->mod_flutterDepthScale = ageFlutterScale * flutterDepthScale;
}
```

**Step 3: Add modifier fields to instance**

```c
float mod_saturationMul;
float mod_wowDepthScale;
float mod_flutterDepthScale;
```

**Step 4: Initialize modifiers**

In v2_create_instance:
```c
inst->mod_saturationMul = 1.0f;
inst->mod_wowDepthScale = 1.0f;
inst->mod_flutterDepthScale = 1.0f;
inst->gs_headroomAdjDb = 0.0f;
v2_ApplyAgeSpeedModifiers(inst);
```

**Step 5: Apply saturation multiplier in TapeSat**

Modify v2_TapeSat_UpdateControls:
```c
inst->sat_factor = Clamp(inst->sat_smoothedDrive * 1.35f * inst->mod_saturationMul, 0.0f, 2.0f);
```

**Step 6: Add parameter handling for age and speed**

In v2_set_param:
```c
} else if (strcmp(key, "age") == 0) {
    inst->param_age = (int)(v * 2.0f + 0.5f);
    v2_ApplyAgeSpeedModifiers(inst);
    v2_GS_SetParams(inst, inst->param_drive, inst->param_color);
} else if (strcmp(key, "speed") == 0) {
    inst->param_speed = (int)(v * 2.0f + 0.5f);
    v2_ApplyAgeSpeedModifiers(inst);
    v2_GS_SetParams(inst, inst->param_drive, inst->param_color);
```

In v2_get_param:
```c
if (strcmp(key, "age") == 0) return snprintf(buf, buf_len, "%d", inst->param_age);
if (strcmp(key, "speed") == 0) return snprintf(buf, buf_len, "%d", inst->param_speed);
```

**Step 7: Build and verify**

Run: `./scripts/build.sh`

**Step 8: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: add tape age and speed modifiers"
```

---

### Task 6: Add Stereo Width (Widen) Control

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Add width processing function**

```c
static void v2_Width_Process(tapescam_instance_t *inst, float *left, float *right, int size) {
    if (!inst->param_widen) return;  /* Skip if widen is off */

    const float width = 1.35f;  /* Default widen amount from VST */

    for (int i = 0; i < size; i++) {
        float mid = 0.5f * (left[i] + right[i]);
        float side = 0.5f * (left[i] - right[i]);
        side *= width;
        left[i] = mid + side;
        right[i] = mid - side;
    }
}
```

**Step 2: Call in signal chain**

In v2_process_block, after compressor and before final output:
```c
v2_Width_Process(inst, left_buf, right_buf, chunk);
```

**Step 3: Add parameter handling**

In v2_set_param:
```c
} else if (strcmp(key, "widen") == 0) {
    inst->param_widen = (v > 0.5f) ? 1 : 0;
```

In v2_get_param:
```c
if (strcmp(key, "widen") == 0) return snprintf(buf, buf_len, "%d", inst->param_widen);
```

**Step 4: Build and verify**

Run: `./scripts/build.sh`

**Step 5: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: add stereo width (widen) control"
```

---

### Task 7: Update module.json with New Parameters

**Files:**
- Modify: `src/module.json`

**Step 1: Update ui_hierarchy and chain_params**

Replace the entire file with:
```json
{
  "id": "tapescam",
  "name": "TAPESCAM",
  "abbrev": "TS",
  "version": "0.4.0",
  "description": "Tape saturation and degradation effect",
  "author": "Charles Vestal",
  "dsp": "tapescam.so",
  "api_version": 2,
  "capabilities": {
    "chainable": true,
    "component_type": "audio_fx",
    "ui_hierarchy": {
      "levels": {
        "root": {
          "name": "TAPESCAM",
          "params": ["input", "drive", "color", "wobble", "noise", "tone", "output"],
          "knobs": ["input", "drive", "color", "wobble", "noise", "tone"],
          "children": ["settings"]
        },
        "settings": {
          "name": "Settings",
          "params": ["age", "speed", "compression", "widen"],
          "knobs": ["age", "speed", "compression", "widen"]
        }
      }
    },
    "chain_params": [
      {"key": "input", "name": "Input", "type": "float", "min": 0.0, "max": 1.0, "default": 0.7, "step": 0.05},
      {"key": "drive", "name": "Drive", "type": "float", "min": 0.0, "max": 1.0, "default": 0.0, "step": 0.05},
      {"key": "color", "name": "Color", "type": "float", "min": 0.0, "max": 1.0, "default": 0.0, "step": 0.05},
      {"key": "wobble", "name": "Wobble", "type": "float", "min": 0.0, "max": 1.0, "default": 0.0, "step": 0.05},
      {"key": "noise", "name": "Noise", "type": "float", "min": 0.0, "max": 1.0, "default": 0.0, "step": 0.05},
      {"key": "tone", "name": "Tone", "type": "float", "min": 0.0, "max": 1.0, "default": 0.5, "step": 0.05},
      {"key": "output", "name": "Output", "type": "float", "min": 0.0, "max": 1.0, "default": 0.7, "step": 0.05},
      {"key": "age", "name": "Age", "type": "enum", "options": ["NEW", "USED", "WORN"], "default": 0},
      {"key": "speed", "name": "Speed", "type": "enum", "options": ["HIGH", "STD", "LOW"], "default": 0},
      {"key": "compression", "name": "Comp", "type": "enum", "options": ["OFF", "LITE", "HEAVY"], "default": 0},
      {"key": "widen", "name": "Widen", "type": "bool", "default": true}
    ]
  }
}
```

Note: Changed output default from 1.0 to 0.7 for safer default level.

**Step 2: Commit**

```bash
git add src/module.json
git commit -m "feat: update module.json with new parameters and settings submenu"
```

---

### Task 8: Update get_param for ui_hierarchy and chain_params

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Update ui_hierarchy JSON**

Replace the ui_hierarchy block in v2_get_param:
```c
if (strcmp(key, "ui_hierarchy") == 0) {
    const char *hierarchy = "{"
        "\"modes\":null,"
        "\"levels\":{"
            "\"root\":{"
                "\"children\":[\"settings\"],"
                "\"knobs\":[\"input\",\"drive\",\"color\",\"wobble\",\"noise\",\"tone\"],"
                "\"params\":[\"input\",\"drive\",\"color\",\"wobble\",\"noise\",\"tone\",\"output\"]"
            "},"
            "\"settings\":{"
                "\"children\":null,"
                "\"knobs\":[\"age\",\"speed\",\"compression\",\"widen\"],"
                "\"params\":[\"age\",\"speed\",\"compression\",\"widen\"]"
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
```

**Step 2: Update chain_params JSON**

```c
if (strcmp(key, "chain_params") == 0) {
    const char *params_json = "["
        "{\"key\":\"input\",\"name\":\"Input\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"drive\",\"name\":\"Drive\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"color\",\"name\":\"Color\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"wobble\",\"name\":\"Wobble\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"noise\",\"name\":\"Noise\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"tone\",\"name\":\"Tone\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"output\",\"name\":\"Output\",\"type\":\"float\",\"min\":0,\"max\":1},"
        "{\"key\":\"age\",\"name\":\"Age\",\"type\":\"enum\",\"options\":[\"NEW\",\"USED\",\"WORN\"]},"
        "{\"key\":\"speed\",\"name\":\"Speed\",\"type\":\"enum\",\"options\":[\"HIGH\",\"STD\",\"LOW\"]},"
        "{\"key\":\"compression\",\"name\":\"Comp\",\"type\":\"enum\",\"options\":[\"OFF\",\"LITE\",\"HEAVY\"]},"
        "{\"key\":\"widen\",\"name\":\"Widen\",\"type\":\"bool\"}"
    "]";
    int len = strlen(params_json);
    if (len < buf_len) {
        strcpy(buf, params_json);
        return len;
    }
    return -1;
}
```

**Step 3: Build and verify**

Run: `./scripts/build.sh`

**Step 4: Commit**

```bash
git add src/dsp/tapescam.c
git commit -m "feat: update get_param with new parameters metadata"
```

---

### Task 9: Final Integration and Testing

**Files:**
- Modify: `src/dsp/tapescam.c`

**Step 1: Verify complete signal chain order**

Ensure v2_process_block has this order:
1. Convert to float with input level
2. v2_TapeSat_UpdateControls, v2_WF_UpdateControls, v2_Tone_UpdateControls, v2_Hiss_UpdateControls
3. v2_GS_Process (gain stage)
4. v2_TapeSat_Process
5. v2_WF_Process (wow/flutter)
6. v2_Tone_Process
7. v2_Hiss_Process (noise - after tone so it gets filtered)
8. v2_Comp_Process (compressor)
9. v2_Width_Process (stereo width)
10. Apply output level + final tanh soft clip
11. Convert back to int16

**Step 2: Full build**

Run: `./scripts/build.sh`
Expected: Clean build with no errors or warnings

**Step 3: Deploy and test**

Run: `./scripts/install.sh`
Test on Move device with various parameter combinations

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat: complete TAPESCAM v0.4.0 with full DSP alignment to TapeScamVST"
```

---

## Summary of Changes

1. **Output level fix**: Moved from inside GainStage to after all processing
2. **Noise module**: Pink/white noise blend based on amount
3. **Compressor module**: 3-mode upward compressor for tape AGC pumping
4. **Age modifier**: NEW/USED/WORN affects headroom, saturation, wow depth
5. **Speed modifier**: HIGH/STD/LOW affects saturation, wow/flutter rates
6. **Widen control**: Mid/side stereo width enhancement
7. **Updated defaults**: Output default changed to 0.7 for safer levels
