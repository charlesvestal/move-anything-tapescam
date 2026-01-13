# CLAUDE.md

Instructions for Claude Code when working with this repository.

## Project Overview

TAPESCAM is an audio effect module for Move Anything that provides tape saturation and degradation effects.

## Architecture

```
src/
  dsp/
    tapescam.c          # Main DSP implementation
    audio_fx_api_v1.h   # Audio FX API (from move-anything)
    plugin_api_v1.h     # Plugin API types (from move-anything)
  module.json           # Module metadata
```

## Key Implementation Details

### Audio FX API

Implements Move Anything audio_fx_api_v1:
- `on_load`: Initialize DSP state
- `on_unload`: Cleanup
- `process_block`: In-place stereo audio processing
- `set_param`: drive, saturation, wobble, tone
- `get_param`: Returns current parameter values

### DSP Chain

1. **Input Gain (Drive)**: Scale input by 1.0 to 4.0x
2. **Soft Clipping (Saturation)**: Normalized tanh waveshaping
3. **Wow/Flutter (Wobble)**: LFO-modulated delay for pitch variation
4. **Tone Filter**: One-pole lowpass (2kHz to 20kHz)

### Signal Chain Integration

Module declares `"chainable": true` and `"component_type": "audio_fx"` in module.json.

Installs to: `/data/UserData/move-anything/modules/chain/audio_fx/tapescam/`

## Build Commands

```bash
./scripts/build.sh      # Build for ARM64 via Docker
./scripts/install.sh    # Deploy to Move
```
