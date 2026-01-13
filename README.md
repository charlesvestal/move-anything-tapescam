# Move Anything TAPESCAM

Tape saturation and degradation audio effect module for Move Anything.

## Features

- **Drive**: Input gain (1x to 4x)
- **Saturation**: Soft clipping via tanh waveshaping
- **Wobble**: LFO-modulated pitch variation (wow/flutter)
- **Tone**: Low-pass filter (2kHz to 20kHz)

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
