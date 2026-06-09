#!/usr/bin/env bash
# Download public bags referenced by benchmarks/scoreboard.json.
# Set BAGX_SCOREBOARD_BAGS (default: .cache/scoreboard-bags under repo root).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${BAGX_SCOREBOARD_BAGS:-$REPO_ROOT/.cache/scoreboard-bags}"
S3_BASE="s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags"

mkdir -p "$OUT_DIR"
cd "$OUT_DIR"

echo "Scoreboard bag cache: $OUT_DIR"

fetch_autoware() {
  local s3_prefix="$1"
  local local_name="$2"
  echo "==> Autoware: $local_name"
  aws s3 sync "${S3_BASE}/${s3_prefix}/" "./${local_name}/" --no-sign-request
}

fetch_autoware "all-sensors-bag1_compressed" "autoware_isuzu_all_sensors_bag1"
fetch_autoware "all-sensors-bag2_compressed" "autoware_isuzu_all_sensors_bag2"
fetch_autoware "all-sensors-bag3_compressed" "autoware_isuzu_all_sensors_bag3"
fetch_autoware "all-sensors-bag5_compressed" "autoware_isuzu_all_sensors_bag5"
fetch_autoware "all-sensors-bag6_compressed" "autoware_isuzu_all_sensors_bag6"
fetch_autoware "driving_30_kmh_2022_06_10-15_47_42_compressed" "driving_30"

if [[ ! -e autocore_os1_64/metadata.yaml ]]; then
  echo "==> AutoCore OS1-64 (~580MB zip)"
  gdown 13oypA5UdlW1XIqw6l5C3qf0I4sFaZ1Nz -O autocore_os1_64.zip
  unzip -q autocore_os1_64.zip -d autocore_os1_64_extracted
  ln -sfn autocore_os1_64_extracted/rosbag2_2022_04_14-ped_vehicle autocore_os1_64
  rm -f autocore_os1_64.zip
fi

echo "Done. Refresh scores with:"
echo "  export BAGX_SCOREBOARD_BAGS=$OUT_DIR"
echo "  export BAGX_DB3_CACHE=$REPO_ROOT/.cache"
echo "  python scripts/generate_scoreboard.py --refresh --write-manifest"
echo "  python scripts/generate_scoreboard.py"
