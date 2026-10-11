#!/usr/bin/env bash
# Download public bags referenced by benchmarks/scoreboard.json.
# Set BAGX_SCOREBOARD_BAGS (default: .cache/scoreboard-bags under repo root).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${BAGX_SCOREBOARD_BAGS:-$REPO_ROOT/.cache/scoreboard-bags}"
CACHE_DIR="${BAGX_DB3_CACHE:-$REPO_ROOT/.cache/db3-decompress}"
S3_BASE="s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags"

mkdir -p "$OUT_DIR" "$CACHE_DIR"
cd "$OUT_DIR"

echo "Scoreboard bag cache: $OUT_DIR"
echo "DB3 decompress cache: $CACHE_DIR"

decompress_zstd_bags() {
  local dir="$1"
  for zstd_file in "$dir"/*.db3.zstd; do
    [[ -f "$zstd_file" ]] || continue
    local out="${zstd_file%.zstd}"
    if [[ ! -f "$out" ]]; then
      echo "  decompress: $(basename "$zstd_file")"
      zstd -d -f "$zstd_file" -o "$out"
    fi
  done
}

fetch_autoware() {
  local s3_prefix="$1"
  local local_name="$2"
  echo "==> Autoware: $local_name"
  aws s3 sync "${S3_BASE}/${s3_prefix}/" "./${local_name}/" --no-sign-request
  decompress_zstd_bags "./${local_name}"
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

if [[ ! -f euroc_mh01/MH_01_easy.bag ]]; then
  echo "==> EuRoC MH_01_easy (HF mirror, ~2.5GB)"
  huggingface-cli download kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01 \
    --local-dir euroc_mh01 --local-dir-use-symlinks False
  ln -sfn euroc_mh01/MH_01_easy.bag MH_01_easy.bag
fi

fetch_zip() {
  local name="$1" url="$2" md5="$3" check="$4"
  if [[ -e "$check" ]]; then
    return
  fi
  echo "==> ${name}"
  curl -fL "$url" -o "${name}.zip"
  echo "${md5}  ${name}.zip" | md5sum -c -
  unzip -q "${name}.zip"
  rm -f "${name}.zip"
}

# Livox MID-360 driving bag (K. Koide / AIST, Zenodo 14841855, CC-BY-4.0, 517MB)
fetch_zip livox_mid360 \
  "https://zenodo.org/api/records/14841855/files/rosbag2_2024_04_16-14_17_01.zip/content" \
  0836c50859bb1af591966b69da166186 livox_mid360_aist/metadata.yaml
if [[ -d rosbag2_2024_04_16-14_17_01 && ! -e livox_mid360_aist ]]; then
  mv rosbag2_2024_04_16-14_17_01 livox_mid360_aist
fi

# NTU VIRAL eee_03 (DR-NTU doi:10.21979/N9/X39LEK, CC BY-NC 4.0, 2.0GB)
fetch_zip eee_03 "https://researchdata.ntu.edu.sg/api/access/datafile/68132" \
  9bf10b5455ee6d2962324ef557f86433 eee_03/eee_03.bag

if [[ ! -f dataset-calib-imu1_512_16.bag ]]; then
  echo "==> TUM VI calib-imu1 (~1.1GB)"
  curl -fL https://cvg.cit.tum.de/tumvi/calibrated/512_16/dataset-calib-imu1_512_16.bag \
    -o dataset-calib-imu1_512_16.bag
fi

fetch_r2b() {
  local seq="$1"
  local base="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/r2bdataset2024/versions/1/files/${seq}"
  if [[ -f "${seq}/${seq}_0.mcap" ]]; then
    return
  fi
  echo "==> NVIDIA r2b: ${seq}"
  mkdir -p "${seq}"
  curl -fL "${base}/metadata.yaml" -o "${seq}/metadata.yaml"
  curl -fL "${base}/${seq}_0.mcap" -o "${seq}/${seq}_0.mcap"
}

for seq in r2b_galileo r2b_galileo2 r2b_whitetunnel r2b_robotarm; do
  fetch_r2b "$seq"
done

# Simulator rows (provenance "simulator") resolve from .cache/dogfood/ captured with
# scripts/run_ros_dogfood.py. Never alias a capture onto a public row's bag name.

echo "Done. Refresh scores with:"
echo "  export BAGX_SCOREBOARD_BAGS=$OUT_DIR"
echo "  export BAGX_DB3_CACHE=$CACHE_DIR"
echo "  python scripts/generate_scoreboard.py --refresh --write-manifest"
echo "  python scripts/generate_scoreboard.py"
