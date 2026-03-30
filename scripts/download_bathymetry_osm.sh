#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${ROOT_DIR}/dataset"
mkdir -p "${DATA_DIR}"

# BBOX order: south,west,north,east
BBOX="${1:-34.05,0.876312,50.4056,34.22}"
OUT_FILE="${2:-${DATA_DIR}/osm_bathymetry_istanbul_genoa.geojson}"
TMP_JSON="$(mktemp)"

OVERPASS_QUERY=$(cat <<EOF
[out:json][timeout:300];
(
  way["seamark:type"~"^(depth_contour|dredged_area|fairway|rock|wreck|obstruction)$"](${BBOX});
  way["seamark:fairway:minimum_depth"](${BBOX});
  way["depth:dredged"](${BBOX});
  way["depth"](${BBOX});
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
import math
import os
import sys

in_path = sys.argv[1]
out_path = sys.argv[2]

with open(in_path, "r", encoding="utf-8") as f:
    data = json.load(f)

elements = data.get("elements", [])
features = []

DEPTH_KEYS = [
    "depth",
    "depth:dredged",
    "seamark:fairway:minimum_depth",
    "seamark:dredged_area:minimum_depth",
    "seamark:obstruction:depth",
    "seamark:wreck:minimum_depth",
    "seamark:rock:depth",
]

def parse_depth(value):
    if value is None:
        return None
    if isinstance(value, (int, float)):
        v = float(value)
        return abs(v) if math.isfinite(v) else None
    if not isinstance(value, str):
        return None

    cleaned = value.strip().lower()
    for suffix in ("m", "meter", "meters", "metre", "metres"):
        if cleaned.endswith(suffix):
            cleaned = cleaned[: -len(suffix)].strip()
            break
    cleaned = cleaned.replace(",", ".")
    if " to " in cleaned:
        cleaned = cleaned.split(" to ", 1)[0].strip()
    if "-" in cleaned and cleaned.count("-") == 1 and not cleaned.startswith("-"):
        cleaned = cleaned.split("-", 1)[0].strip()
    try:
        v = float(cleaned)
    except ValueError:
        return None
    return abs(v) if math.isfinite(v) else None

def is_closed(coords):
    if len(coords) < 4:
        return False
    a = coords[0]
    b = coords[-1]
    return abs(a[0] - b[0]) < 1e-12 and abs(a[1] - b[1]) < 1e-12

def extract_depth(tags):
    if not isinstance(tags, dict):
        return None, None
    for key in DEPTH_KEYS:
        if key in tags:
            depth = parse_depth(tags.get(key))
            if depth is not None:
                return depth, key
    return None, None

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
    seamark_type = tags.get("seamark:type", "")
    depth_m, depth_key = extract_depth(tags)
    closed = is_closed(coords)

    if closed:
        geometry = {"type": "Polygon", "coordinates": [coords]}
    else:
        geometry = {"type": "LineString", "coordinates": coords}

    props = {
        "id": el.get("id"),
        "feature_type": seamark_type or "bathymetry",
    }
    if depth_m is not None:
        props["depth_m"] = depth_m
        props["depth_source_key"] = depth_key
    if isinstance(tags, dict):
        if "name" in tags:
            props["name"] = tags["name"]
        if "seamark:name" in tags:
            props["seamark:name"] = tags["seamark:name"]

    features.append({
        "type": "Feature",
        "geometry": geometry,
        "properties": props,
    })

os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w", encoding="utf-8") as f:
    json.dump({"type": "FeatureCollection", "features": features}, f, ensure_ascii=False)

print(f"Wrote {len(features)} bathymetry features to {out_path}")
PY

rm -f "${TMP_JSON}"
echo "Done: ${OUT_FILE}"
