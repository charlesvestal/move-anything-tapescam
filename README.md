# Schwung TAPESCAM

Tape saturation and degradation audio effect module for Schwung.

## Prerequisites

- [Schwung](https://github.com/charlesvestal/schwung) installed on your Ableton Move

## Installation

### Via Module Store (Recommended)

1. Launch Schwung on your Move
2. Select **Module Store** from the main menu
3. Navigate to **Audio FX** → **TAPESCAM**
4. Select **Install**

### Manual Installation

```bash
./scripts/install.sh
```

## Features

- **Input**: Input level (pre-gain)
- **Drive**: Distortion/clipping amount
- **Color**: Saturation character (affects both drive stage and tape saturation)
- **Wobble**: Wow/flutter amount (tape pitch modulation)
- **Tone**: Tilt EQ (dark to bright)
- **Output**: Output level

## Building

```bash
./scripts/build.sh
```

## Installing

```bash
./scripts/install.sh
```

The module will be installed to `/data/UserData/schwung/modules/chain/audio_fx/tapescam/` on your Move.

## Usage

This is an audio effect module for use within Signal Chain. Add it after a sound generator in your chain preset.

## Credits

Ported from [CVCHothouse](https://github.com/charlesvestal/CVCHothouse) (Daisy Seed effects pedal collection).

Inspired by analog tape saturation characteristics.

## License

MIT License - see LICENSE file for details.

## AI Assistance Disclaimer

This module is part of Schwung and was developed with AI assistance, including Claude, Codex, and other AI assistants.

All architecture, implementation, and release decisions are reviewed by human maintainers.  
AI-assisted content may still contain errors, so please validate functionality, security, and license compatibility before production use.
