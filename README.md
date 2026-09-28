# lac_symphonie — VRX digital twin of Lac Symphonie (Metz)

A VRX / Gazebo Harmonic world of Lac Symphonie, the lake in front of the Georgia
Tech Europe building on the Metz Technopôle, built from public data:

| what | source |
|---|---|
| shoreline + island | OpenStreetMap relation 8841690 (ODbL) |
| terrain | IGN RGE ALTI 1 m DTM through the Géoplateforme WMS (Etalab licence) |
| ground texture | IGN orthophoto, 20 cm |
| trees | OSM `natural=tree` / `tree_row` / `wood` + the IGN surface model (MNS − DTM) for the lakeside canopy and tree heights |
| tree meshes | Gazebo Fuel `OpenRobotics/Oak tree` and `Pine Tree` |
| lake bed | synthetic bowl (max 2.5 m) — no public bathymetry |

World origin (Gazebo world frame = ROS `map`, ENU): **49.1035229 N, 6.2171120 E,
191.74 m** (lake centroid at water level). z = 0 is the water surface. The `navsat`
sensor of any vehicle therefore reports real coordinates.

## Run

```bash
cd ~/workspaces/vrx_ws && source /opt/ros/jazzy/setup.bash
colcon build --merge-install --packages-select lac_symphonie && source install/setup.bash

ros2 launch lac_symphonie lac_symphonie.launch.py            # Kingfisher + geo_tf_publisher (+ headless:=True)
ros2 launch vrx_gz competition.launch.py world:=lac_symphonie \
     config_file:=$(ros2 pkg prefix lac_symphonie)/share/lac_symphonie/config/lac_symphonie_spawn.yaml
```

Checks: `ros2 topic echo --once /kingfisher/sensors/gps/gps/fix` → ≈ 49.1024 N, 6.2173 E;
`ros2 run tf2_ros tf2_echo map georgia_tech` → (−145.2, −154.4, 0).

### Wind

`vrx::USVWind` gives the lake one uniform wind. Set it per launch; it is calm by default:

| arg | default | meaning |
|---|---|---|
| `wind_speed` | 0.0 | mean speed, m/s |
| `wind_direction` | 240 | where the wind goes, degrees counter-clockwise from east |
| `wind_gust` | 0.0 | gust gain, about the standard deviation of the speed, m/s |

e.g. `ros2 launch lac_symphonie lac_symphonie.launch.py wind_speed:=5 wind_direction:=0` blows toward the east.
The launch writes a copy of the world with these values into a temp dir, first on `GZ_SIM_RESOURCE_PATH`.
The true wind is on `/vrx/debug/wind/{speed,direction}`. Hull windage applies to the models listed in the
world's `USVWind` block (`wamv`, `kingfisher`).

## Lat/lon → TF: `geo_tf_publisher`

Publishes one static transform `map → <name>` per point. Origin is read from the
world SDF (`world_sdf` parameter) so it can never drift from the simulation.

- `config/geo_points.yaml`: `{name, latitude, longitude, altitude?, yaw_deg?}` entries
  (altitude absolute, default = water level).
- runtime: `ros2 topic pub --once /geo_tf_publisher/add_point geographic_msgs/msg/GeoPointStamped
  "{header: {frame_id: pontoon}, position: {latitude: 49.1030, longitude: 6.2160, altitude: .nan}}"`
- CLI: `scripts/latlon_to_local.py LAT LON` / `--reverse X Y` (spawn poses, waypoints).

Conversion uses `gz.math7.SphericalCoordinates` with the `LOCAL2` frame — what gz-sim's
navsat uses. (`LOCAL` / `local_from_spherical_position` carries a known 2 % scale bug and a
sign flip in the inverse; the VRX scoring plugins use that path.) A pure-Python WGS-84
fallback (`lac_symphonie/geo.py`) is used without the ROS environment; `test/test_geo.py`
asserts both agree to < 1 cm.

`map → base_link` of the vehicle comes from a localization stack as on the real robot
(see the workspace README §5). For quick checks bridge the ground truth:
`ros2 run ros_gz_bridge parameter_bridge /model/kingfisher/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V --ros-args -r /model/kingfisher/ground_truth/tf:=/tf`.

## Regenerating the world (`scripts/`)

Needs network once; python3 with numpy, scipy, Pillow, matplotlib (all in the ROS image).

```bash
python3 scripts/fetch_data.py       # -> data/ (git-ignored): OSM json, DTM/MNS float32, orthophoto
python3 scripts/gen_terrain.py      # -> models/lac_symphonie_terrain, models/lac_symphonie_water, data/terrain_preview.png
python3 scripts/gen_trees.py        # -> models/tree_* wrappers, splices <include>s into worlds/lac_symphonie.sdf, data/trees_preview.png
```

Knobs at the top of each script: bbox and resolutions (`fetch_data.py`), mesh spacing,
bowl depth/slope, water margin (`gen_terrain.py`), row/fill spacing, shore band width,
tree cap, size classes (`gen_trees.py`). `gen_terrain.py --bathymetry lat,lon,depth.csv`
interpolates real soundings instead of the bowl. `gen_terrain.py --clamp` raises land to
≥ 0.3 m for use with the stock `coast_waves` sea plane (swap the include in the world).

Everything between `<!-- BEGIN generated trees -->` / `<!-- END generated trees -->` in
the world file is rewritten by `gen_trees.py`; lines carrying `<!-- manual -->` inside the
block are kept, everything outside the block is hand-editable.

## Manual steps / known gaps

1. Spawn point and landmarks: pick lat/lon on `data/ortho.jpg` (north up, bbox in
   `fetch_data.py`) or Google Earth, convert with `latlon_to_local.py`, edit
   `config/lac_symphonie_spawn.yaml` and `config/geo_points.yaml`.
2. Bathymetry: the bed is a bowl; feed real soundings with `--bathymetry`.
3. Higher-resolution terrain: LiDAR HD MNT 50 cm tiles from https://cartes.gouv.fr can
   replace `data/dtm.bin` (convert to a float32 raster on the same bbox first).
4. Buildings are only in the ground texture; extruding OSM footprints is a possible next step.
5. Tree species: OSM has no `leaf_type` tags here, so every tree is an oak; edit the
   generated block or `species_of()` in `gen_trees.py`.
6. Performance: 600 tree meshes + a 230k-triangle terrain run at RTF 1.0 on an RTX 3080;
   on weaker GPUs lower `TREE_CAP` and raise `VISUAL_STEP_M`.
