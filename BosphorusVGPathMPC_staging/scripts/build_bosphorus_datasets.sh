#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILDER="${ROOT_DIR}/build/buffered_land_dataset_builder"

INPUT="${ROOT_DIR}/dataset/osm_land_istanbul_genoa.geojson"

MIN_LAT="40.85"
MAX_LAT="41.35"
MIN_LON="28.80"
MAX_LON="29.35"

POLYGON_OUTPUT="${ROOT_DIR}/dataset/bosphorus/polygon/land_polygon.geojson"
MESH_OUTPUT="${ROOT_DIR}/dataset/bosphorus/mesh/land_mesh.geojson"
BUFFER_OUTPUT_DIR="${ROOT_DIR}/dataset/bosphorus/buffered"
BAND_OUTPUT="${ROOT_DIR}/dataset/bosphorus/buffered/sea_bands.geojson"
BASENAME="bosphorus_land"

MESH_SIMPLIFY_M="20"
MESH_POST_SIMPLIFY_M="30"
MESH_MIN_AREA_M2="5000"
BUFFER_SIMPLIFY_M="0"

usage() {
  cat <<EOF
Kullanim:
  $(basename "$0") [secenekler]

Varsayilan olarak Istanbul Bogazi icin:
  - yerel polygon dataset
  - 50/100/150/200 m buffer datasetleri + sea bands
  - yerel mesh dataset
uretir.

Secenekler:
  --input <path>                  Varsayilan: ${INPUT}
  --min-lat <deg>                 Varsayilan: ${MIN_LAT}
  --max-lat <deg>                 Varsayilan: ${MAX_LAT}
  --min-lon <deg>                 Varsayilan: ${MIN_LON}
  --max-lon <deg>                 Varsayilan: ${MAX_LON}
  --polygon-output <path>         Varsayilan: ${POLYGON_OUTPUT}
  --mesh-output <path>            Varsayilan: ${MESH_OUTPUT}
  --buffer-output-dir <dir>       Varsayilan: ${BUFFER_OUTPUT_DIR}
  --band-output <path>            Varsayilan: ${BAND_OUTPUT}
  --mesh-simplify-m <metre>       Varsayilan: ${MESH_SIMPLIFY_M}
  --mesh-post-simplify-m <metre>  Varsayilan: ${MESH_POST_SIMPLIFY_M}
  --mesh-min-area-m2 <m2>         Varsayilan: ${MESH_MIN_AREA_M2}
  --buffer-simplify-m <metre>     Varsayilan: ${BUFFER_SIMPLIFY_M}
  -h, --help                      Yardim goster
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)
      INPUT="$2"
      shift 2
      ;;
    --min-lat)
      MIN_LAT="$2"
      shift 2
      ;;
    --max-lat)
      MAX_LAT="$2"
      shift 2
      ;;
    --min-lon)
      MIN_LON="$2"
      shift 2
      ;;
    --max-lon)
      MAX_LON="$2"
      shift 2
      ;;
    --polygon-output)
      POLYGON_OUTPUT="$2"
      shift 2
      ;;
    --mesh-output)
      MESH_OUTPUT="$2"
      shift 2
      ;;
    --buffer-output-dir)
      BUFFER_OUTPUT_DIR="$2"
      shift 2
      ;;
    --band-output)
      BAND_OUTPUT="$2"
      shift 2
      ;;
    --mesh-simplify-m)
      MESH_SIMPLIFY_M="$2"
      shift 2
      ;;
    --mesh-post-simplify-m)
      MESH_POST_SIMPLIFY_M="$2"
      shift 2
      ;;
    --mesh-min-area-m2)
      MESH_MIN_AREA_M2="$2"
      shift 2
      ;;
    --buffer-simplify-m)
      BUFFER_SIMPLIFY_M="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Bilinmeyen arguman: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "${BUILDER}" ]]; then
  echo "Builder bulunamadi: ${BUILDER}" >&2
  echo "Once projeyi build edin." >&2
  exit 1
fi

mkdir -p "$(dirname "${POLYGON_OUTPUT}")"
mkdir -p "$(dirname "${MESH_OUTPUT}")"
mkdir -p "${BUFFER_OUTPUT_DIR}"

echo "Istanbul Bogazi veri altyapisi uretiliyor..."
echo "Input : ${INPUT}"
echo "Bounds: lat ${MIN_LAT}..${MAX_LAT}, lon ${MIN_LON}..${MAX_LON}"

echo
echo "[1/3] Yerel polygon dataset"
"${BUILDER}" \
  --input "${INPUT}" \
  --offset-m 0 \
  --output "${POLYGON_OUTPUT}" \
  --simplify-m 0 \
  --min-lat "${MIN_LAT}" \
  --max-lat "${MAX_LAT}" \
  --min-lon "${MIN_LON}" \
  --max-lon "${MAX_LON}"

echo
echo "[2/3] Buffer datasetleri ve sea bands"
"${BUILDER}" \
  --input "${INPUT}" \
  --offsets-m 50,100,150,200 \
  --output-dir "${BUFFER_OUTPUT_DIR}" \
  --band-output "${BAND_OUTPUT}" \
  --basename "${BASENAME}" \
  --simplify-m "${BUFFER_SIMPLIFY_M}" \
  --min-lat "${MIN_LAT}" \
  --max-lat "${MAX_LAT}" \
  --min-lon "${MIN_LON}" \
  --max-lon "${MAX_LON}"

echo
echo "[3/3] Yerel mesh dataset"
"${BUILDER}" \
  --input "${INPUT}" \
  --offset-m 0 \
  --mesh-output "${MESH_OUTPUT}" \
  --simplify-m "${MESH_SIMPLIFY_M}" \
  --mesh-post-simplify-m "${MESH_POST_SIMPLIFY_M}" \
  --mesh-min-area-m2 "${MESH_MIN_AREA_M2}" \
  --min-lat "${MIN_LAT}" \
  --max-lat "${MAX_LAT}" \
  --min-lon "${MIN_LON}" \
  --max-lon "${MAX_LON}"

echo
echo "Hazir:"
echo "  Polygon: ${POLYGON_OUTPUT}"
echo "  Mesh   : ${MESH_OUTPUT}"
echo "  Buffers: ${BUFFER_OUTPUT_DIR}"
echo "  Bands  : ${BAND_OUTPUT}"
