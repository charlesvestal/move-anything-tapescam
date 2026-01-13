# Move Anything TAPESCAM

Tape saturation and degradation audio effect module for Move Anything.

## Prerequisites

- [Move Anything](https://github.com/charlesvestal/move-anything) installed on your Ableton Move

## Installation

### Via Module Store (Recommended)

1. Launch Move Anything on your Move
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

The module will be installed to `/data/UserData/move-anything/modules/chain/audio_fx/tapescam/` on your Move.

## Usage

This is an audio effect module for use within Signal Chain. Add it after a sound generator in your chain preset.

## Credits

Ported from [CVCHothouse](https://github.com/charlesvestal/CVCHothouse) (Daisy Seed effects pedal collection).

Inspired by analog tape saturation characteristics.

## License

MIT License - see LICENSE file for details.
