#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP_DIR="${ROOT_DIR}/.tmp_osm_land"
ZIP_URL="${OSM_LAND_ZIP_URL:-https://osmdata.openstreetmap.de/download/land-polygons-split-4326.zip}"
ZIP_FILE="${TMP_DIR}/land-polygons-split-4326.zip"
EXTRACT_DIR="${TMP_DIR}/land-polygons-split-4326"

mkdir -p "${TMP_DIR}" "${EXTRACT_DIR}"

if [ ! -f "${ZIP_FILE}" ]; then
  echo "OSM world land polygons indiriliyor: ${ZIP_URL}"
  curl -L --fail --retry 3 -o "${ZIP_FILE}" "${ZIP_URL}"
else
  echo "Zip zaten var, indirme atlandi: ${ZIP_FILE}"
fi

echo "Zip aciliyor..."
unzip -o -q "${ZIP_FILE}" -d "${EXTRACT_DIR}"

SHP_FILE="$(find "${EXTRACT_DIR}" -type f -name 'land_polygons.shp' | head -n 1)"
if [ -z "${SHP_FILE}" ]; then
  echo "land_polygons.shp bulunamadi." >&2
  exit 1
fi

echo "Hazir dataset kaynagi: ${SHP_FILE}"
