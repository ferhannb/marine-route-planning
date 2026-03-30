#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${ROOT_DIR}/dataset"
mkdir -p "${DATA_DIR}"

# BBOX order: south,west,north,east
BBOX="${1:-34.05,0.876312,50.4056,34.22}"
OUT_FILE="${2:-${DATA_DIR}/osm_tss_istanbul_genoa.geojson}"
TMP_JSON="$(mktemp)"

OVERPASS_QUERY=$(cat <<EOF
[out:json][timeout:240];
(
  way["seamark:type"~"^(separation_lane|separation_line|separation_boundary|separation_zone)$"](${BBOX});
);
out body geom;
EOF
)

fetch_overpass() {
  local endpoint="$1"
  curl -fsSL --data-urlencode "data=${OVERPASS_QUERY}" "${endpoint}" -o "${TMP_JSON}"
}

if ! fetch_overpass "https://overpass-api.de/api/interpreter"; then
  echo "Primary Overpass endpoint failed, trying fallback..."
  fetch_overpass "https://overpass.kumi.systems/api/interpreter"
fi

python3 - "${TMP_JSON}" "${OUT_FILE}" <<'PY'
import json
import sys

in_path = sys.argv[1]
out_path = sys.argv[2]

with open(in_path, "r", encoding="utf-8") as f:
    data = json.load(f)

elements = data.get("elements", [])
features = []

def seamark_type(tags):
    if not isinstance(tags, dict):
        return "tss"
    return tags.get("seamark:type", "tss")

def is_closed(coords):
    if len(coords) < 4:
        return False
    a = coords[0]
    b = coords[-1]
    return abs(a[0] - b[0]) < 1e-12 and abs(a[1] - b[1]) < 1e-12

for el in elements:
    if el.get("type") != "way":
        continue
    geom = el.get("geometry")
    if not isinstance(geom, list) or len(geom) < 2:
        continue
    coords = []
    bad = False
    for p in geom:
        if not isinstance(p, dict) or "lon" not in p or "lat" not in p:
            bad = True
            break
        coords.append([float(p["lon"]), float(p["lat"])])
    if bad or len(coords) < 2:
        continue

    tags = el.get("tags", {})
    stype = seamark_type(tags)
    props = {
        "id": el.get("id"),
        "seamark:type": stype,
    }
    if isinstance(tags, dict) and "name" in tags:
        props["name"] = tags["name"]

    if stype == "separation_zone" and is_closed(coords):
        geometry = {"type": "Polygon", "coordinates": [coords]}
    elif is_closed(coords):
        geometry = {"type": "Polygon", "coordinates": [coords]}
    else:
        geometry = {"type": "LineString", "coordinates": coords}

    features.append({
        "type": "Feature",
        "geometry": geometry,
        "properties": props,
    })

fc = {"type": "FeatureCollection", "features": features}
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(fc, f, ensure_ascii=False)

print(f"Wrote {len(features)} TSS features to {out_path}")
PY

rm -f "${TMP_JSON}"
echo "Done: ${OUT_FILE}"
