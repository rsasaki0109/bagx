#!/usr/bin/env bash
# Verify bagx ROS1 support against a public TUM VI bag (or local .bag path).
set -euo pipefail

BAG_URL="https://cvg.cit.tum.de/tumvi/calibrated/512_16/dataset-calib-imu1_512_16.bag"
WORKDIR="${WORKDIR:-/tmp/bagx-ros1-verify}"
BAG_PATH="${1:-$WORKDIR/tumvi-calib-imu1.bag}"

mkdir -p "$WORKDIR"

if [[ ! -f "$BAG_PATH" ]]; then
  echo "Downloading TUM VI calib bag (~1 GB)..."
  wget --continue -O "$BAG_PATH" "$BAG_URL"
fi

echo "Running bagx eval on $BAG_PATH"
bagx eval "$BAG_PATH" --json "$WORKDIR/tumvi-eval.json"
