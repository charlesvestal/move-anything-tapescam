#!/bin/bash
# Install TAPESCAM module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

if [ ! -d "dist/tapescam" ]; then
    echo "Error: dist/tapescam not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing TAPESCAM Module ==="

# Deploy to Move (audio_fx path)
echo "Copying module to Move..."
ssh ableton@move.local "mkdir -p /data/UserData/move-anything/modules/chain/audio_fx/tapescam"
scp -r dist/tapescam/* ableton@move.local:/data/UserData/move-anything/modules/chain/audio_fx/tapescam/

echo ""
echo "=== Install Complete ==="
echo "Module installed to: /data/UserData/move-anything/modules/chain/audio_fx/tapescam/"
echo ""
echo "Restart Move Anything to load the new module."
