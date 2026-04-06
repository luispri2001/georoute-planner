"""Waypoint and POI loading helpers."""

import json


def load_waypoints_bis(filepath="data/points.geojson"):
    """Load waypoints/POIs from a GeoJSON file on disk."""
    with open(filepath, "r", encoding="utf-8") as file_obj:
        geojson = json.load(file_obj)

    return geojson


def load_waypoints():
    """Return embedded example POIs in GeoJSON-like format."""
    geojson = {
        "type": "FeatureCollection",
        "features": [
            {"id": "pastizal_1", "type": "Feature", "properties": {"landcover": "pastizal"}, "geometry": {"type": "Point", "coordinates": [-6.207228, 42.310665]}},
            {"id": "pastizal_2", "type": "Feature", "properties": {"landcover": "pastizal"}, "geometry": {"type": "Point", "coordinates": [-6.201653, 42.30961]}},
            {"id": "pastizal_3", "type": "Feature", "properties": {"landcover": "pastizal"}, "geometry": {"type": "Point", "coordinates": [-6.20393, 42.310527]}},
            {"id": "arbustivo_1", "type": "Feature", "properties": {"landcover": "arbustivo"}, "geometry": {"type": "Point", "coordinates": [-6.204549, 42.31036]}},
            {"id": "arbustivo_2", "type": "Feature", "properties": {"landcover": "arbustivo"}, "geometry": {"type": "Point", "coordinates": [-6.206416, 42.310902]}},
            {"id": "matorral_1", "type": "Feature", "properties": {"landcover": "matorral"}, "geometry": {"type": "Point", "coordinates": [-6.203042, 42.310832]}},
            {"id": "matorral_2", "type": "Feature", "properties": {"landcover": "matorral"}, "geometry": {"type": "Point", "coordinates": [-6.205465, 42.311749]}},
            {"id": "water_1", "type": "Feature", "properties": {"landcover": "water"}, "geometry": {"type": "Point", "coordinates": [-6.204025, 42.309282]}},
            {"id": "water_2", "type": "Feature", "properties": {"landcover": "water"}, "geometry": {"type": "Point", "coordinates": [-6.204347, 42.312561]}},
        ],
    }
    return geojson


def get_point_by_id(geojson, feature_id):
    """Return a point as [lat, lon] by feature ID, or None if missing."""
    for feature in geojson["features"]:
        if feature["id"] == feature_id:
            lon, lat = feature["geometry"]["coordinates"]
            return [lat, lon]
    return None
