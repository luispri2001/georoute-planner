"""Costmap-based routing utilities (grid creation, landcover costs, A* pathfinding)."""

import heapq

import numpy as np
import osmnx as ox


LANDCOVER_COST = {
    "road": 1,
    "track": 1.5,
    "pastizal": 10,
    "arbustivo": 30,
    "matorral": 80,
    "water": 1000,
}


def create_cost_grid(geojson, waypoints, resolution=2):
    """Create a cost grid that covers waypoints and all GeoJSON points."""
    all_lats = [p[0] for p in waypoints]
    all_lons = [p[1] for p in waypoints]

    for feature in geojson["features"]:
        lon, lat = feature["geometry"]["coordinates"]
        all_lats.append(lat)
        all_lons.append(lon)

    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lon, max_lon = min(all_lons), max(all_lons)

    # Small padding so border points stay within the grid.
    padding = 0.0005
    min_lat -= padding
    max_lat += padding
    min_lon -= padding
    max_lon += padding

    height = int((max_lat - min_lat) * 111000 / resolution)
    width = int((max_lon - min_lon) * 85000 / resolution)

    grid = np.ones((height, width)) * 10
    return grid, min_lat, min_lon, resolution


def apply_landcover_costs(grid, geojson, min_lat, min_lon, resolution):
    """Apply per-cell cost overrides from point landcover labels."""
    for feature in geojson["features"]:
        lon, lat = feature["geometry"]["coordinates"]
        landcover = feature["properties"]["landcover"]

        y = int((lat - min_lat) * 111000 / resolution)
        x = int((lon - min_lon) * 85000 / resolution)

        if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
            grid[y, x] = LANDCOVER_COST.get(landcover, 10)

    return grid


def astar(grid, start, goal):
    """Run A* on a 2D cost grid, allowing 8-connected movement."""
    neighbors = [
        (1, 0), (0, 1), (-1, 0), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    ]

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    gscore = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy in neighbors:
            nxp = current[0] + dx
            nyp = current[1] + dy

            if 0 <= nxp < grid.shape[0] and 0 <= nyp < grid.shape[1]:
                step_cost = np.sqrt(dx * dx + dy * dy)
                cost = grid[nxp, nyp] * step_cost
                tentative = gscore[current] + cost

                if tentative < gscore.get((nxp, nyp), 1e9):
                    came_from[(nxp, nyp)] = current
                    gscore[(nxp, nyp)] = tentative

                    heuristic = np.linalg.norm(np.array([nxp, nyp]) - np.array(goal))
                    heapq.heappush(open_set, (tentative + heuristic, (nxp, nyp)))

    return []


def smooth_path(path):
    """Drop intermediate points that keep the same direction vector."""
    if len(path) < 3:
        return path

    smoothed = [path[0]]

    for i in range(1, len(path) - 1):
        prev_point = np.array(path[i - 1])
        curr_point = np.array(path[i])
        next_point = np.array(path[i + 1])

        v1 = curr_point - prev_point
        v2 = next_point - curr_point

        if not np.array_equal(v1, v2):
            smoothed.append(tuple(curr_point))

    smoothed.append(path[-1])
    return smoothed


def gps_to_grid(lat, lon, min_lat, min_lon, resolution):
    """Convert [lat, lon] to grid coordinates [row, col]."""
    y = int((lat - min_lat) * 111000 / resolution)
    x = int((lon - min_lon) * 85000 / resolution)
    return (y, x)


def grid_to_gps(path, min_lat, min_lon, resolution):
    """Convert a grid path [row, col] back to [lat, lon]."""
    gps_points = []

    for y, x in path:
        lat = min_lat + y * resolution / 111000
        lon = min_lon + x * resolution / 85000
        gps_points.append([lat, lon])

    return gps_points


def add_osm_tracks(grid, min_lat, min_lon, resolution, center, dist):
    """Project nearby OSM track/path edges onto the local cost grid."""
    custom_filter = '["highway"~"track|path|service|unclassified"]'

    graph = ox.graph_from_point(center, dist=dist, custom_filter=custom_filter)

    for _, _, data in graph.edges(data=True):
        if "geometry" not in data:
            continue

        xs, ys = data["geometry"].xy

        for i in range(len(xs) - 1):
            lon1, lat1 = xs[i], ys[i]
            lon2, lat2 = xs[i + 1], ys[i + 1]

            y1 = int((lat1 - min_lat) * 111000 / resolution)
            x1 = int((lon1 - min_lon) * 85000 / resolution)
            y2 = int((lat2 - min_lat) * 111000 / resolution)
            x2 = int((lon2 - min_lon) * 85000 / resolution)

            steps = max(abs(x2 - x1), abs(y2 - y1)) + 1

            for step in range(steps):
                x = int(x1 + (x2 - x1) * step / steps)
                y = int(y1 + (y2 - y1) * step / steps)

                if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
                    grid[y, x] = LANDCOVER_COST["track"]


def generate_costmap_route(geojson, waypoints):
    """Generate a route between ordered waypoints using costmap+A*."""
    grid, min_lat, min_lon, resolution = create_cost_grid(geojson, waypoints)

    add_osm_tracks(grid, min_lat, min_lon, resolution, waypoints[0], 600)
    grid = apply_landcover_costs(grid, geojson, min_lat, min_lon, resolution)

    full_path = []

    for i in range(len(waypoints) - 1):
        start = gps_to_grid(waypoints[i][0], waypoints[i][1], min_lat, min_lon, resolution)
        goal = gps_to_grid(waypoints[i + 1][0], waypoints[i + 1][1], min_lat, min_lon, resolution)

        path = smooth_path(astar(grid, start, goal))

        if i > 0:
            full_path.extend(path[1:])
        else:
            full_path.extend(path)

    return grid_to_gps(full_path, min_lat, min_lon, resolution)
