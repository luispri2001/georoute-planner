"""Waypoint and POI loading helpers."""

import json
from pathlib import Path


def load_waypoints(filepath="data/points.geojson"):
    """Load waypoints/POIs from a GeoJSON file on disk."""
    path = Path(filepath)
    if not path.is_absolute() and not path.exists():
        # Resolve default relative paths from project root for notebook/script callers.
        project_root = Path(__file__).resolve().parents[1]
        path = project_root / path

    with open(path, "r", encoding="utf-8") as file_obj:
        geojson = json.load(file_obj)

    return geojson



def get_point_by_id(geojson, feature_id):
    """Return a point as [lat, lon] by feature ID, or None if missing."""
    for feature in geojson["features"]:
        if feature["id"] == feature_id:
            lon, lat = feature["geometry"]["coordinates"]
            return [lat, lon]
    return None
