"""Direct and OSMnx-based route generation helpers."""

from geopy import distance
import networkx as nx
import numpy as np
import osmnx as ox


def interpolate_points(p1, p2, interval_m=10):
    """Interpolate intermediate points between p1 and p2 each interval_m meters."""
    dist_m = distance.distance(p1, p2).m

    if dist_m <= interval_m:
        return [list(p1), list(p2)]

    n_points = int(dist_m // interval_m)
    lats = np.linspace(p1[0], p2[0], n_points + 2)
    lons = np.linspace(p1[1], p2[1], n_points + 2)

    return [list(point) for point in zip(lats, lons)]


def generate_robot_route(waypoints, interval_m=10):
    """Generate a direct (straight-line) high-resolution route across waypoints."""
    full_route = []

    for i in range(len(waypoints) - 1):
        segment = interpolate_points(waypoints[i], waypoints[i + 1], interval_m)

        if i > 0:
            full_route.extend(segment[1:])
        else:
            full_route.extend(segment)

    return full_route


def generate_osmnx_route_bis(waypoints, network_type="all", interval_m=10):
    """Legacy OSMnx route generator; keeps behavior for backward compatibility."""
    lats = [p[0] for p in waypoints]
    lons = [p[1] for p in waypoints]

    center = (sum(lats) / len(lats), sum(lons) / len(lons))

    max_dist = 0
    for point in waypoints:
        dist = distance.distance(center, point).m
        if dist > max_dist:
            max_dist = dist

    dist_m = max_dist + 500

    print(f"Fetching road network around center {center} with radius {dist_m}m...")
    ox.settings.max_query_area_size = 2500000000

    try:
        graph = ox.graph_from_point(center, dist=dist_m, network_type=network_type, simplify=True)
        nodes = ox.nearest_nodes(graph, lons, lats)

        full_path_coords = []

        for i in range(len(nodes) - 1):
            path = nx.shortest_path(graph, nodes[i], nodes[i + 1], weight="length")
            segment_coords = [[graph.nodes[n]["y"], graph.nodes[n]["x"]] for n in path]

            if i > 0:
                full_path_coords.extend(segment_coords[1:])
            else:
                full_path_coords.extend(segment_coords)

        high_res_route = []

        for i in range(len(full_path_coords) - 1):
            segment = interpolate_points(full_path_coords[i], full_path_coords[i + 1], interval_m)
            if i > 0:
                high_res_route.extend(segment[1:])
            else:
                high_res_route.extend(segment)

        return high_res_route

    except Exception as exc:  # pragma: no cover - fallback path depends on network/OSM response.
        print(f"OSMnx routing failed: {exc}. Falling back to straight-line interpolation.")
        return generate_robot_route(waypoints, interval_m)


def generate_osmnx_route(waypoints, network_type="all", interval_m=10):
    """Generate an OSM road-following route and densify points by interval_m."""
    lats = [p[0] for p in waypoints]
    lons = [p[1] for p in waypoints]

    center = (sum(lats) / len(lats), sum(lons) / len(lons))

    max_dist = 0
    for point in waypoints:
        dist = distance.distance(center, point).m
        if dist > max_dist:
            max_dist = dist

    dist_m = max_dist + 500

    print(f"Fetching road network around center {center} with radius {dist_m}m...")
    ox.settings.max_query_area_size = 2500000000

    try:
        graph = ox.graph_from_point(center, dist=dist_m, network_type=network_type, simplify=True)
        nodes = ox.nearest_nodes(graph, lons, lats)

        full_path_coords = []

        for i in range(len(nodes) - 1):
            path = nx.shortest_path(graph, nodes[i], nodes[i + 1], weight="length")
            route_gdf = ox.routing.route_to_gdf(graph, path)

            segment_coords = []
            for geometry in route_gdf.geometry:
                if geometry.geom_type == "LineString":
                    xs, ys = geometry.xy
                    for lon, lat in zip(xs, ys):
                        segment_coords.append([lat, lon])
                elif geometry.geom_type == "MultiLineString":
                    for part in geometry:
                        xs, ys = part.xy
                        for lon, lat in zip(xs, ys):
                            segment_coords.append([lat, lon])

            if i > 0:
                full_path_coords.extend(segment_coords[1:])
            else:
                full_path_coords.extend(segment_coords)

        high_res_route = []

        for i in range(len(full_path_coords) - 1):
            segment = interpolate_points(full_path_coords[i], full_path_coords[i + 1], interval_m)
            if i > 0:
                high_res_route.extend(segment[1:])
            else:
                high_res_route.extend(segment)

        return high_res_route

    except Exception as exc:  # pragma: no cover - fallback path depends on network/OSM response.
        print(f"OSMnx routing failed: {exc}. Falling back to straight-line interpolation.")
        return generate_robot_route(waypoints, interval_m)
