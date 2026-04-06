"""Interactive waypoint selector UI for GeoRoute Planner.

Run with:
    streamlit run examples/waypoint_selector_app.py
"""

import json
import os
from contextlib import contextmanager
from pathlib import Path
import sys

import pandas as pd
import pydeck as pdk
import streamlit as st


@contextmanager
def temporary_cwd(target_dir: Path):
    """Temporarily switch working directory for deterministic output paths."""
    previous_dir = Path.cwd()
    os.chdir(target_dir)
    try:
        yield
    finally:
        os.chdir(previous_dir)


def resolve_project_root() -> Path:
    """Find repository root from current working directory or this file location."""
    cwd = Path.cwd().resolve()
    if (cwd / "src").exists() and (cwd / "data").exists():
        return cwd
    if (cwd.parent / "src").exists() and (cwd.parent / "data").exists():
        return cwd.parent
    return Path(__file__).resolve().parents[1]


def load_geojson_points(geojson_path: Path) -> pd.DataFrame:
    """Load point features from GeoJSON into a DataFrame."""
    with open(geojson_path, "r", encoding="utf-8") as file_obj:
        geojson = json.load(file_obj)

    rows = []
    for feature in geojson.get("features", []):
        if feature.get("geometry", {}).get("type") != "Point":
            continue
        lon, lat = feature["geometry"]["coordinates"]
        rows.append(
            {
                "id": feature.get("id", "unknown"),
                "landcover": feature.get("properties", {}).get("landcover", "unknown"),
                "lat": lat,
                "lon": lon,
            }
        )

    if not rows:
        raise ValueError("No point features found in the selected GeoJSON file.")

    return pd.DataFrame(rows).sort_values("id").reset_index(drop=True)


def build_map_layers(df: pd.DataFrame, selected_route_ids: list[str]) -> tuple[list, pdk.ViewState]:
    """Create PyDeck layers for all points and selected route points."""
    color_map = {
        "pastizal": [34, 139, 34],
        "arbustivo": [255, 140, 0],
        "matorral": [138, 43, 226],
        "water": [30, 144, 255],
    }

    map_df = df.copy()
    map_df["color"] = map_df["landcover"].map(color_map).apply(
        lambda c: c if isinstance(c, list) else [128, 128, 128]
    )

    selected_df = pd.DataFrame(columns=["id", "lat", "lon", "order_label"])
    if selected_route_ids:
        selected_df = map_df.set_index("id").loc[selected_route_ids].reset_index()[["id", "lat", "lon"]]
        selected_df["order_label"] = [str(i + 1) for i in range(len(selected_df))]

    all_points_layer = pdk.Layer(
        "ScatterplotLayer",
        map_df,
        get_position="[lon, lat]",
        get_radius=12,
        radius_min_pixels=5,
        radius_max_pixels=12,
        get_fill_color="color",
        pickable=True,
    )

    selected_points_layer = pdk.Layer(
        "ScatterplotLayer",
        selected_df,
        get_position="[lon, lat]",
        get_radius=18,
        radius_min_pixels=8,
        radius_max_pixels=18,
        get_fill_color=[220, 20, 60],
        get_line_color=[255, 255, 255],
        line_width_min_pixels=2,
        stroked=True,
        pickable=True,
    )

    order_text_layer = pdk.Layer(
        "TextLayer",
        selected_df,
        get_position="[lon, lat]",
        get_text="order_label",
        get_size=16,
        get_color=[255, 255, 255],
        get_alignment_baseline="center",
        pickable=False,
    )

    view_state = pdk.ViewState(
        latitude=float(map_df["lat"].mean()),
        longitude=float(map_df["lon"].mean()),
        zoom=14,
        pitch=0,
    )

    return [all_points_layer, selected_points_layer, order_text_layer], view_state


def main() -> None:
    """Run Streamlit waypoint selector app."""
    st.set_page_config(page_title="GeoRoute Waypoint Selector", layout="wide")
    st.title("GeoRoute Waypoint Selector")
    st.caption("Visualize available points and choose the waypoint visit order.")

    project_root = resolve_project_root()
    src_dir = project_root / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))

    st.sidebar.header("Data Source")
    geojson_input = st.sidebar.text_input("GeoJSON path", value="data/points.geojson")

    geojson_path = Path(geojson_input)
    if not geojson_path.is_absolute():
        geojson_path = project_root / geojson_path

    if not geojson_path.exists():
        st.error(f"GeoJSON file not found: {geojson_path}")
        st.stop()

    df = load_geojson_points(geojson_path)

    if "route_ids" not in st.session_state:
        st.session_state.route_ids = []

    available_ids = df["id"].tolist()

    st.subheader("Available Points")
    st.dataframe(df[["id", "landcover", "lat", "lon"]], use_container_width=True, hide_index=True)

    col_left, col_right = st.columns([2, 1])

    with col_right:
        st.subheader("Route Builder")
        selected_id = st.selectbox("Choose waypoint", options=available_ids, index=None, placeholder="Select an ID")

        add_clicked = st.button("Add waypoint to route", use_container_width=True)
        if add_clicked and selected_id:
            if selected_id in st.session_state.route_ids:
                st.warning(f"{selected_id} is already in the route.")
            else:
                st.session_state.route_ids.append(selected_id)

        remove_id = st.selectbox(
            "Remove waypoint",
            options=st.session_state.route_ids,
            index=None,
            placeholder="Select route ID",
        )
        remove_clicked = st.button("Remove selected waypoint", use_container_width=True)
        if remove_clicked and remove_id:
            st.session_state.route_ids = [rid for rid in st.session_state.route_ids if rid != remove_id]

        if st.button("Clear route", use_container_width=True):
            st.session_state.route_ids = []

        st.markdown("**Current visit order**")
        if st.session_state.route_ids:
            route_table = pd.DataFrame(
                {
                    "order": list(range(1, len(st.session_state.route_ids) + 1)),
                    "id": st.session_state.route_ids,
                }
            )

            edited_route_table = st.data_editor(
                route_table,
                use_container_width=True,
                hide_index=True,
                key="route_order_editor",
                column_config={
                    "order": st.column_config.NumberColumn(
                        "order",
                        min_value=1,
                        step=1,
                        help="Lower number means earlier visit.",
                    ),
                    "id": st.column_config.SelectboxColumn(
                        "id",
                        options=available_ids,
                        required=True,
                        help="Select waypoint ID for each route step.",
                    ),
                },
            )

            if st.button("Apply order/ID changes", use_container_width=True):
                candidate = edited_route_table.copy()
                candidate["order"] = pd.to_numeric(candidate["order"], errors="coerce")

                if candidate["order"].isna().any():
                    st.error("All route steps must have a valid numeric order.")
                else:
                    candidate = candidate.sort_values("order").reset_index(drop=True)
                    updated_ids = candidate["id"].astype(str).tolist()

                    if len(set(updated_ids)) != len(updated_ids):
                        st.error("Waypoint IDs in the route must be unique.")
                    elif any(route_id not in available_ids for route_id in updated_ids):
                        st.error("One or more waypoint IDs are not available in the GeoJSON file.")
                    else:
                        st.session_state.route_ids = updated_ids
                        st.success("Visit order updated.")
        else:
            st.info("No waypoints selected yet.")

        route_payload = {
            "filepath": str(geojson_path),
            "route_ids": st.session_state.route_ids,
        }
        st.download_button(
            label="Download selected order (JSON)",
            data=json.dumps(route_payload, indent=2),
            file_name="selected_route_order.json",
            mime="application/json",
            use_container_width=True,
        )

        st.markdown("**Use in route generation**")
        st.code(
            "import robot_route_generation as rrg\n"
            f"rrg.main(filepath=\"{geojson_input}\", route_ids={st.session_state.route_ids})",
            language="python",
        )

        run_clicked = st.button("Generate route now", type="primary", use_container_width=True)
        if run_clicked:
            if not st.session_state.route_ids:
                st.error("Select at least one waypoint before generating the route.")
            else:
                with st.spinner("Generating routes and map..."):
                    try:
                        import robot_route_generation as rrg

                        # Run from project root so routes/ and maps/ are always updated in repo paths.
                        with temporary_cwd(project_root):
                            rrg.main(filepath=str(geojson_path), route_ids=list(st.session_state.route_ids))
                        st.success("Route generation completed.")
                    except Exception as exc:
                        st.exception(exc)

    with col_left:
        st.subheader("Map")
        layers, view_state = build_map_layers(df, st.session_state.route_ids)
        tooltip = {
            "html": "<b>ID:</b> {id}<br/><b>Landcover:</b> {landcover}",
            "style": {"backgroundColor": "#222", "color": "white"},
        }
        deck = pdk.Deck(
            map_provider="carto",
            map_style="light",
            initial_view_state=view_state,
            layers=layers,
            tooltip=tooltip,
        )
        st.pydeck_chart(deck, use_container_width=True)

        map_dir = project_root / "maps"
        latest_map = None
        if map_dir.exists():
            html_maps = sorted(map_dir.glob("*.html"), key=lambda p: p.stat().st_mtime, reverse=True)
            if html_maps:
                latest_map = html_maps[0]

        if latest_map is not None:
            with st.expander("Show generated route map preview", expanded=True):
                map_html = latest_map.read_text(encoding="utf-8")
                map_html = map_html + f"\n<!-- preview_ts:{latest_map.stat().st_mtime_ns} -->"
                st.components.v1.html(map_html, height=650, scrolling=True)
                st.caption(f"Map file: {latest_map}")


if __name__ == "__main__":
    main()
