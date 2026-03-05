import os

import folium
from geopy import distance
import numpy as np
import pandas as pd
import osmnx as ox
import networkx as nx



LANDCOVER_COST = {
    "road": 1,
    "track": 1.5,
    "pastizal": 10,
    "arbustivo": 30,
    "matorral": 80,
    "water": 1000
}


def create_cost_grid(geojson, waypoints, resolution=2):
    """
    Creates a cost grid covering the bounding box of waypoints and POIs.
    resolution = meters per cell
    """
    
    all_lats = [p[0] for p in waypoints]
    all_lons = [p[1] for p in waypoints]

    for f in geojson["features"]:
        lon, lat = f["geometry"]["coordinates"]
        all_lats.append(lat)
        all_lons.append(lon)

    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lon, max_lon = min(all_lons), max(all_lons)

    # padding
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

    for feature in geojson["features"]:

        lon, lat = feature["geometry"]["coordinates"]
        lc = feature["properties"]["landcover"]

        y = int((lat - min_lat) * 111000 / resolution)
        x = int((lon - min_lon) * 85000 / resolution)

        if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
            grid[y, x] = LANDCOVER_COST.get(lc, 10)

    return grid

import heapq

def astar(grid, start, goal):

    neighbors = [
        (1,0),(0,1),(-1,0),(0,-1),
        (1,1),(1,-1),(-1,1),(-1,-1)
    ]

    open_set=[]
    heapq.heappush(open_set,(0,start))

    came_from={}
    gscore={start:0}

    while open_set:

        _,current=heapq.heappop(open_set)

        if current==goal:

            path=[]
            while current in came_from:
                path.append(current)
                current=came_from[current]

            return path[::-1]

        for dx,dy in neighbors:

            nxp=current[0]+dx
            nyp=current[1]+dy

            if 0<=nxp<grid.shape[0] and 0<=nyp<grid.shape[1]:

                step_cost=np.sqrt(dx*dx+dy*dy)
                cost=grid[nxp,nyp]*step_cost

                tentative=gscore[current]+cost

                if tentative < gscore.get((nxp,nyp),1e9):

                    came_from[(nxp,nyp)]=current
                    gscore[(nxp,nyp)]=tentative

                    h=np.linalg.norm(
                        np.array([nxp,nyp])-np.array(goal)
                    )

                    heapq.heappush(
                        open_set,
                        (tentative+h,(nxp,nyp))
                    )

    return []

def smooth_path(path):

    if len(path) < 3:
        return path

    smooth=[path[0]]

    for i in range(1,len(path)-1):

        prev=np.array(path[i-1])
        curr=np.array(path[i])
        nxt=np.array(path[i+1])

        v1=curr-prev
        v2=nxt-curr

        if not np.array_equal(v1,v2):
            smooth.append(tuple(curr))

    smooth.append(path[-1])

    return smooth

def gps_to_grid(lat, lon, min_lat, min_lon, resolution):

    y=int((lat-min_lat)*111000/resolution)
    x=int((lon-min_lon)*85000/resolution)

    return (y,x)


def grid_to_gps(path, min_lat, min_lon, resolution):

    gps=[]

    for y,x in path:

        lat=min_lat+y*resolution/111000
        lon=min_lon+x*resolution/85000

        gps.append([lat,lon])

    return gps


def add_osm_tracks(grid, min_lat, min_lon, resolution, center, dist):

    custom_filter = '["highway"~"track|path|service|unclassified"]'

    G = ox.graph_from_point(
        center,
        dist=dist,
        custom_filter=custom_filter
    )

    for u, v, data in G.edges(data=True):

        if "geometry" in data:

            xs, ys = data["geometry"].xy

            for i in range(len(xs)-1):

                lon1, lat1 = xs[i], ys[i]
                lon2, lat2 = xs[i+1], ys[i+1]

                y1 = int((lat1-min_lat)*111000/resolution)
                x1 = int((lon1-min_lon)*85000/resolution)

                y2 = int((lat2-min_lat)*111000/resolution)
                x2 = int((lon2-min_lon)*85000/resolution)

                steps = max(abs(x2-x1), abs(y2-y1)) + 1

                for s in range(steps):

                    x = int(x1 + (x2-x1)*s/steps)
                    y = int(y1 + (y2-y1)*s/steps)

                    if 0<=y<grid.shape[0] and 0<=x<grid.shape[1]:
                        grid[y,x] = LANDCOVER_COST["track"]


def generate_costmap_route(geojson, waypoints):

    grid,min_lat,min_lon,res=create_cost_grid(geojson,waypoints)

    center = waypoints[0]

    add_osm_tracks(
        grid,
        min_lat,
        min_lon,
        res,
        center,
        600
    )

    grid=apply_landcover_costs(grid,geojson,min_lat,min_lon,res)

    full_path=[]

    for i in range(len(waypoints)-1):

        start=gps_to_grid(waypoints[i][0],waypoints[i][1],min_lat,min_lon,res)
        goal=gps_to_grid(waypoints[i+1][0],waypoints[i+1][1],min_lat,min_lon,res)

        path = smooth_path(astar(grid,start,goal))

        if i>0:
            full_path.extend(path[1:])
        else:
            full_path.extend(path)

    return grid_to_gps(full_path,min_lat,min_lon,res)


import json

def load_waypoints_bis(filepath="data/points.geojson"):
    """
    Load GeoJSON waypoints from a file.
    """
    with open(filepath, "r") as f:
        geojson = json.load(f)

    return geojson



def load_waypoints():
    """
    Returns a GeoJSON-like dictionary of categorized points of interest (POIs).
    Note: GeoJSON coordinates are [longitude, latitude].
    """
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
            {"id": "water_2", "type": "Feature", "properties": {"landcover": "water"}, "geometry": {"type": "Point", "coordinates": [-6.204347, 42.312561]}}
        ]
    }
    return geojson

def get_point_by_id(geojson, feature_id):
    """Helper to get [lat, lon] from GeoJSON by ID."""
    for feature in geojson["features"]:
        if feature["id"] == feature_id:
            lon, lat = feature["geometry"]["coordinates"]
            return [lat, lon]
    return None

def interpolate_points(p1, p2, interval_m=10):
    """
    Interpolate points between two GPS coordinates p1 [lat, lon] and p2 [lat, lon]
    every interval_m meters.
    """
    dist = distance.distance(p1, p2).m
    if dist <= interval_m:
        return [list(p1), list(p2)]
    
    n_points = int(dist // interval_m)
    lats = np.linspace(p1[0], p2[0], n_points + 2)
    lons = np.linspace(p1[1], p2[1], n_points + 2)
    
    return [list(p) for p in zip(lats, lons)]

def generate_robot_route(waypoints, interval_m=10):
    """
    Takes an ordered list of waypoints [lat, lon] and returns a high-resolution 
    route with points separated by interval_m.
    """
    full_route = []
    for i in range(len(waypoints) - 1):
        segment = interpolate_points(waypoints[i], waypoints[i+1], interval_m)
        if i > 0:
            full_route.extend(segment[1:])
        else:
            full_route.extend(segment)
    return full_route

def generate_osmnx_route_bis(waypoints, network_type='all', interval_m=10):
    """
    Generates a route that follows roads using OSMnx, then interpolates 
    every interval_m meters. Expects waypoints as list of [lat, lon].
    """
    # 1. Get the center and radius for routing
    lats = [p[0] for p in waypoints]
    lons = [p[1] for p in waypoints]
    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    center = (center_lat, center_lon)
    
    print("OSM nodes:", nodes)

    # Calculate approx distance to cover all points + padding
    max_dist = 0
    for p in waypoints:
        d = distance.distance(center, p).m
        if d > max_dist:
            max_dist = d
    
    dist_m = max_dist + 500 # 500m padding
    
    print(f"Fetching road network around center {center} with radius {dist_m}m...")
    
    # Configure OSMnx
    ox.settings.max_query_area_size = 2500000000  # 2500 sq km
    
    try:
        # 2. Download the road network
        G = ox.graph_from_point(center, dist=dist_m, network_type=network_type, simplify=True)
        
        # 3. Find the nearest nodes to each waypoint (ox expects lon, lat here internally or handles it)
        # nearest_nodes in recent versions takes (G, X, Y) where X=lons, Y=lats
        nodes = ox.nearest_nodes(G, lons, lats)
        
        full_path_coords = []
        
        # 4. Find shortest path between consecutive waypoints
        for i in range(len(nodes) - 1):
            source_node = nodes[i]
            target_node = nodes[i+1]
            
            # Find the shortest path (by length)
            path = nx.shortest_path(G, source_node, target_node, weight='length')
            
            # Convert path nodes to coordinates [lat, lon]
            segment_coords = [[G.nodes[n]['y'], G.nodes[n]['x']] for n in path]
            
            if i > 0:
                full_path_coords.extend(segment_coords[1:])
            else:
                full_path_coords.extend(segment_coords)
        
        # 5. Interpolate the road-based path every interval_m meters
        high_res_route = []
        for i in range(len(full_path_coords) - 1):
            seg = interpolate_points(full_path_coords[i], full_path_coords[i+1], interval_m)
            if i > 0:
                high_res_route.extend(seg[1:])
            else:
                high_res_route.extend(seg)
                
        return high_res_route
        
    except Exception as e:
        print(f"OSMnx routing failed: {e}. Falling back to straight-line interpolation.")
        return generate_robot_route(waypoints, interval_m)

def generate_osmnx_route(waypoints, network_type='all', interval_m=10):
    """
    Generates a route that follows roads using OSMnx edge geometries,
    then interpolates every interval_m meters.
    Expects waypoints as list of [lat, lon].
    """

    # 1. Get the center and radius for routing
    lats = [p[0] for p in waypoints]
    lons = [p[1] for p in waypoints]

    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    center = (center_lat, center_lon)

    max_dist = 0
    for p in waypoints:
        d = distance.distance(center, p).m
        if d > max_dist:
            max_dist = d

    dist_m = max_dist + 500

    print(f"Fetching road network around center {center} with radius {dist_m}m...")

    ox.settings.max_query_area_size = 2500000000


    
    try:

        # 2. Download the road network
        G = ox.graph_from_point(
            center,
            dist=dist_m,
            network_type=network_type,
            simplify=True
        )

        # 3. Snap waypoints to nearest nodes
        nodes = ox.nearest_nodes(G, lons, lats)

        full_path_coords = []

        # 4. Route between consecutive waypoints
        for i in range(len(nodes) - 1):

            source_node = nodes[i]
            target_node = nodes[i + 1]

            path = nx.shortest_path(
                G,
                source_node,
                target_node,
                weight='length'
            )

            # 🔵 THIS IS THE IMPORTANT PART
            route_gdf = ox.routing.route_to_gdf(G, path)

            segment_coords = []

            for geom in route_gdf.geometry:

                if geom.geom_type == "LineString":

                    xs, ys = geom.xy

                    for lon, lat in zip(xs, ys):
                        segment_coords.append([lat, lon])

                elif geom.geom_type == "MultiLineString":

                    for part in geom:
                        xs, ys = part.xy
                        for lon, lat in zip(xs, ys):
                            segment_coords.append([lat, lon])

            if i > 0:
                full_path_coords.extend(segment_coords[1:])
            else:
                full_path_coords.extend(segment_coords)

        # 5. Interpolate every interval_m meters
        high_res_route = []

        for i in range(len(full_path_coords) - 1):

            seg = interpolate_points(
                full_path_coords[i],
                full_path_coords[i + 1],
                interval_m
            )

            if i > 0:
                high_res_route.extend(seg[1:])
            else:
                high_res_route.extend(seg)

        return high_res_route

    except Exception as e:

        print(f"OSMnx routing failed: {e}. Falling back to straight-line interpolation.")

        return generate_robot_route(waypoints, interval_m)


def main():
    # 1. Load the waypoints (GeoJSON format)
    # poi_geojson = load_waypoints("data/points.geojson")
    poi_geojson = load_waypoints()
    
    # 2. Define the order of the points to visit using IDs
    # This matches the custom example requested: water_1 -> arbustivo_2 -> water_2
    route_ids = ["water_1", "arbustivo_2", "water_2"]
    robot_waypoints = [get_point_by_id(poi_geojson, rid) for rid in route_ids]
    
    # Filter out None if any ID was not found
    robot_waypoints = [p for p in robot_waypoints if p]

    print(f"Visit order: {route_ids}")
    print(f"Generating routes for {len(robot_waypoints)} waypoints...")

    # 3. Generate BOTH routes: Direct and Road-based
    interval = 10
    
    print("\n--- Option A: Direct Route (Straight Lines) ---")
    direct_route = generate_robot_route(robot_waypoints, interval_m=interval)
    print(f"Direct Route: {len(direct_route)} points, approx {round(len(direct_route)*interval, 2)}m")

    print("\n--- Option B: Road-based Route (OSMnx) ---")
    road_route = generate_osmnx_route(robot_waypoints, network_type='walk', interval_m=interval)
    print(f"Road Route: {len(road_route)} points, approx {round(len(road_route)*interval, 2)}m")

    print("\n--- Option C: Costmap Route (A*) ---")
    cost_route = generate_costmap_route(poi_geojson, robot_waypoints)
    print(f"Cost Route: {len(cost_route)} points")


    # 4. Create a Folium map for visualization
    # Use Esri World Imagery (Satellite)
    satellite_url = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    satellite_attr = "&copy; Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EBP, and the GIS User Community"
    
    m = folium.Map(location=robot_waypoints[0], 
                   zoom_start=17, 
                   tiles=satellite_url, 
                   attr=satellite_attr)

    # Visualization - Categorized POIs markers
    colors = {"pastizal": "green", "arbustivo": "orange", "matorral": "purple", "water": "blue"}
    for feature in poi_geojson["features"]:
        lon, lat = feature["geometry"]["coordinates"]
        category = feature["properties"]["landcover"]
        fid = feature["id"]
        folium.Marker(
            location=[lat, lon],
            popup=f"ID: {fid}<br>Landcover: {category}",
            icon=folium.Icon(color=colors.get(category, "gray"), icon='info-sign')
        ).add_to(m)

    # Visualization - Direct Route (dashed red line)
    folium.PolyLine(direct_route, color="red", weight=2, opacity=0.5, dash_array='5, 5', tooltip="Direct Route").add_to(m)

    # Visualization - Road Route (solid blue line)
    folium.PolyLine(road_route, color="blue", weight=4, opacity=0.8, tooltip="Road-following Route").add_to(m)

    folium.PolyLine(
        cost_route,
        color="green",
        weight=4,
        opacity=0.8,
        tooltip="Costmap Route"
    ).add_to(m)


    # direct → radius=1
    # osm    → radius=2
    # cost   → radius=3

    # Visualization - Individual Route Points (Small circles)
    for p in road_route:
        folium.CircleMarker(location=p, radius=2, color="blue", fill=True, fill_opacity=0.7).add_to(m)
    for p in direct_route:
        folium.CircleMarker(location=p, radius=1, color="red", fill=True, fill_opacity=0.5).add_to(m)
    for p in cost_route:
        folium.CircleMarker(location=p, radius=3, color="green", fill=True, fill_opacity=0.7).add_to(m)

    # Save the map
    map_output = os.path.join("maps", "robot_route_satellite_points.html")
    os.makedirs("maps", exist_ok=True)
    m.save(map_output)
    print(f"\nMap with dual options and points saved to {map_output}")

    # --- New: Export both CSVs ---
    # 1. Road Route CSV
    road_csv = os.path.join("routes", "route_road_osmnx.csv")
    pd.DataFrame(road_route, columns=["latitude", "longitude"]).to_csv(road_csv, index=False)
    
    # 2. Direct Route CSV
    direct_csv = os.path.join("routes", "route_direct_straight.csv")
    pd.DataFrame(direct_route, columns=["latitude", "longitude"]).to_csv(direct_csv, index=False)
    
    cost_csv = os.path.join("routes", "route_costmap_astar.csv")
    pd.DataFrame(cost_route, columns=["latitude","longitude"]).to_csv(cost_csv,index=False)

    print(f"Exported Costmap Route to: {cost_csv}({len(road_route)} points)")
    print(f"Exported Road Route to: {road_csv} ({len(road_route)} points)")
    print(f"Exported Direct Route to: {direct_csv} ({len(direct_route)} points)")

if __name__ == "__main__":
    main()
