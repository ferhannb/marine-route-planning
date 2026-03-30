#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${ROOT_DIR}/dataset"
mkdir -p "${DATA_DIR}"

download_if_missing() {
  local url="$1"
  local out="$2"
  if [[ -f "${out}" ]]; then
    echo "Skip (exists): ${out}"
    return
  fi
  echo "Downloading: ${url}"
  curl -fL "${url}" -o "${out}"
}

download_if_missing \
  "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_land.geojson" \
  "${DATA_DIR}/ne_110m_land.geojson"

download_if_missing \
  "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_50m_land.geojson" \
  "${DATA_DIR}/ne_50m_land.geojson"

download_if_missing \
  "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_land.geojson" \
  "${DATA_DIR}/ne_10m_land.geojson"

echo "Done. Files are in ${DATA_DIR}"
