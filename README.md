# AURORAS GeoRoute Planner

Geographic route generation tools for outdoor robots developed in the AURORAS project.

Project website:
https://project-auroras.github.io/auroras/

## Overview

This repository generates robot routes from geographic waypoints using three strategies:

1. Direct GPS route
Straight-line interpolation between waypoints.

2. OSM road-based route
Uses OpenStreetMap road and track data through OSMnx.

3. Terrain-aware route
Uses a costmap and A* path planning with landcover penalties.

Routes can be exported as CSV files and visualized on a satellite map.

## Repository Structure

```text
georoute-planner
├── README.md
├── LICENSE
├── requirements.txt
├── requirements_venv1.txt
├── data/
│   └── points.geojson
├── examples/
│   ├── generate_route_with_poses.py
│   └── test_poses.py
├── maps/
│   └── robot_route_satellite_points.html (generated)
├── routes/ (generated CSV files)
└── src/
    ├── robot_route_generation.py
    ├── route_generation.py
    ├── costmap_routing.py
    ├── pose_generation.py
    └── waypoint_data.py
```

## Setup

Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

If you are using the alternate dependency set, use:

```bash
pip install -r requirements_venv1.txt
```

## Running Examples

Do you need the files in examples/?

- No, for normal use you only need the main generator: src/robot_route_generation.py
- Yes, if you want a lightweight pose-only example or a quick pose validation script

Run all three routing strategies (direct, OSM road, costmap A*) and generate map and CSV outputs:

```bash
python src/robot_route_generation.py
```

Run simplified direct-route + pose export example (no OSM routing):

```bash
python examples/generate_route_with_poses.py
```

Run pose generation test script:

```bash
python examples/test_poses.py
```

Run interactive waypoint selector UI (choose visit order visually):

```bash
streamlit run examples/waypoint_selector_app.py
```

The app shows available points from GeoJSON on a map and lets you build an ordered waypoint list for route generation.

## Example Outputs

Main script outputs:

- routes/route_direct_straight.csv
- routes/route_road_osmnx.csv
- routes/route_costmap_astar.csv
- routes/route_direct_straight_poses.csv
- routes/route_road_osmnx_poses.csv
- routes/route_costmap_astar_poses.csv
- maps/robot_route_satellite_points.html

Simplified pose example outputs:

- routes/robot_route_legacy.csv
- routes/robot_route_with_poses.csv

Pose test output:

- routes/test_poses.csv

## Visualization

The generated map overlays:

- direct route
- OSM road-following route
- terrain-aware costmap route

Each route is shown over satellite imagery and can also be consumed from CSV files.

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

Full license text:
https://www.gnu.org/licenses/gpl-3.0.html

## Acknowledgements

Developed as part of the AURORAS project.

Project page:
https://project-auroras.github.io/auroras/