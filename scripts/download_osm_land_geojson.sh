#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${ROOT_DIR}/dataset"
TMP_DIR="${ROOT_DIR}/.tmp_osm_land"
ZIP_URL="${OSM_LAND_ZIP_URL:-https://osmdata.openstreetmap.de/download/land-polygons-split-4326.zip}"
ZIP_FILE="${TMP_DIR}/land-polygons-split-4326.zip"
EXTRACT_DIR="${TMP_DIR}/land-polygons-split-4326"
OUT_FILE="${1:-${DATA_DIR}/osm_land_istanbul_genoa.geojson}"
BBOX="${2:-8.9,34.0,29.1,45.0}" # min_lon,min_lat,max_lon,max_lat
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "${DATA_DIR}" "${TMP_DIR}" "${EXTRACT_DIR}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "Python bulunamadi: ${PYTHON_BIN}" >&2
  exit 1
fi

if ! "${PYTHON_BIN}" - <<'PY' >/dev/null 2>&1
import importlib.util
import sys
sys.exit(0 if importlib.util.find_spec("shapefile") else 1)
PY
then
  echo "Python modulu 'shapefile' (pyshp) eksik. Kurulum: ${PYTHON_BIN} -m pip install pyshp" >&2
  exit 1
fi

if [ ! -f "${ZIP_FILE}" ]; then
  echo "OSM land polygons indiriliyor: ${ZIP_URL}"
  curl -L --fail --retry 3 -o "${ZIP_FILE}" "${ZIP_URL}"
else
  echo "Zip zaten var, indirme atlandi: ${ZIP_FILE}"
fi

echo "Zip aciliyor..."
unzip -o -q "${ZIP_FILE}" -d "${EXTRACT_DIR}"

SHP_FILE="$(find "${EXTRACT_DIR}" -type f -name '*.shp' | head -n 1)"
if [ -z "${SHP_FILE}" ]; then
  echo "Shapefile bulunamadi." >&2
  exit 1
fi

echo "GeoJSON donusumu basladi: ${SHP_FILE}"
"${PYTHON_BIN}" - "${SHP_FILE}" "${OUT_FILE}" "${BBOX}" <<'PY'
import json
import os
import sys
import shapefile

shp_path = sys.argv[1]
out_path = sys.argv[2]
bbox_raw = sys.argv[3]

try:
    min_lon, min_lat, max_lon, max_lat = [float(x) for x in bbox_raw.split(",")]
except Exception as exc:
    raise SystemExit(f"Gecersiz bbox: {bbox_raw} ({exc})")

reader = shapefile.Reader(shp_path)

def bbox_overlaps(b):
    # shapefile bbox: [xmin, ymin, xmax, ymax]
    return not (b[2] < min_lon or b[0] > max_lon or b[3] < min_lat or b[1] > max_lat)

features = []
for shp in reader.iterShapes():
    if not bbox_overlaps(shp.bbox):
        continue
    points = shp.points
    parts = list(shp.parts) + [len(points)]
    rings = []
    for i in range(len(parts) - 1):
        ring = points[parts[i]:parts[i + 1]]
        if len(ring) < 4:
            continue
        # close ring if needed
        if ring[0] != ring[-1]:
            ring = ring + [ring[0]]
        rings.append(ring)
    if not rings:
        continue
    # keep as Polygon with all rings; parser currently uses outer ring only.
    geom = {"type": "Polygon", "coordinates": [rings[0]]}
    features.append({"type": "Feature", "properties": {}, "geometry": geom})

os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w", encoding="utf-8") as f:
    json.dump({"type": "FeatureCollection", "features": features}, f, separators=(",", ":"))

print(f"Wrote {len(features)} features to {out_path}")
PY

echo "Tamamlandi: ${OUT_FILE}"
