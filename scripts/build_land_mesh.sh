#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILDER="${ROOT_DIR}/build/buffered_land_dataset_builder"

INPUT="${ROOT_DIR}/dataset/osm_land_istanbul_genoa.geojson"
OUTPUT="${ROOT_DIR}/dataset/mesh/land_mesh.geojson"
OFFSET_M="0"
SIMPLIFY_M="75"
MESH_POST_SIMPLIFY_M="125"
MESH_MIN_AREA_M2="250000"
MIN_LAT="34.0"
MAX_LAT="51.0"
MIN_LON="0.0"
MAX_LON="35.0"

usage() {
  cat <<EOF
Kullanim:
  $(basename "$0") [secenekler]

Varsayilan olarak, triangulation icin sade bir land mesh dataset uretir ve
config/route_planner_mesh.ini dosyasinin kullandigi hedefe yazar.

Secenekler:
  --input <path>                  Varsayilan: ${INPUT}
  --output <path>                 Varsayilan: ${OUTPUT}
  --offset-m <metre>              Varsayilan: ${OFFSET_M}
  --simplify-m <metre>            Varsayilan: ${SIMPLIFY_M}
  --mesh-post-simplify-m <metre>  Varsayilan: ${MESH_POST_SIMPLIFY_M}
  --mesh-min-area-m2 <m2>         Varsayilan: ${MESH_MIN_AREA_M2}
  --min-lat <deg>                 Varsayilan: ${MIN_LAT}
  --max-lat <deg>                 Varsayilan: ${MAX_LAT}
  --min-lon <deg>                 Varsayilan: ${MIN_LON}
  --max-lon <deg>                 Varsayilan: ${MAX_LON}
  -h, --help                      Yardim goster

Ornek:
  $(basename "$0") \\
    --output "${ROOT_DIR}/dataset/mesh/my_region_dp_mesh.geojson" \\
    --simplify-m 60 \\
    --mesh-post-simplify-m 90 \\
    --mesh-min-area-m2 100000 \\
    --min-lat 40.3 --max-lat 41.7 --min-lon 27.8 --max-lon 30.2
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)
      INPUT="$2"
      shift 2
      ;;
    --output)
      OUTPUT="$2"
      shift 2
      ;;
    --offset-m)
      OFFSET_M="$2"
      shift 2
      ;;
    --simplify-m)
      SIMPLIFY_M="$2"
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

mkdir -p "$(dirname "${OUTPUT}")"

echo "Triangulation mesh dataset uretiliyor..."
echo "Input : ${INPUT}"
echo "Output: ${OUTPUT}"
echo "Bounds: lat ${MIN_LAT}..${MAX_LAT}, lon ${MIN_LON}..${MAX_LON}"
echo "Simplify / post / min-area: ${SIMPLIFY_M} m / ${MESH_POST_SIMPLIFY_M} m / ${MESH_MIN_AREA_M2} m2"

"${BUILDER}" \
  --input "${INPUT}" \
  --offset-m "${OFFSET_M}" \
  --mesh-output "${OUTPUT}" \
  --simplify-m "${SIMPLIFY_M}" \
  --mesh-post-simplify-m "${MESH_POST_SIMPLIFY_M}" \
  --mesh-min-area-m2 "${MESH_MIN_AREA_M2}" \
  --min-lat "${MIN_LAT}" \
  --max-lat "${MAX_LAT}" \
  --min-lon "${MIN_LON}" \
  --max-lon "${MAX_LON}"

echo "Hazir: ${OUTPUT}"
