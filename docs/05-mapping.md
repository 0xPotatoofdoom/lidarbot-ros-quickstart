# 5. Mapping with slam_toolbox

```bash
./up slam
```

This runs slam_toolbox in online async mode with the settings in
[config/slam_toolbox.yaml](../config/slam_toolbox.yaml). Yahboom's defaults cap the
lidar range at 2 m, which leaves big rooms mostly unmapped; this config uses 8 m and
adds a new scan every 0.2 m or 0.3 rad of travel. The file is linked into the
container, so edits take effect on the next `./up slam` with no rebuild.

Watch `/map` build up in Foxglove's 3D panel with the fixed frame set to `map`.

## Driving to map

Drive slowly. Wheel slip and quick turns make the odometry drift faster than the scan
matcher can correct it. Some tips:

- A 360-degree spin in place adds nothing: the lidar already sees all around.
- Move into doorways and around furniture so the lidar sees behind things.
- Loop back to where you started, so slam_toolbox can close the loop.

### Semi-automatic exploration

```bash
./ros python3 /work/tools/explore.py 6        # up to 6 steps
```

Each step scores 16 headings by how much unknown map they'd reveal, skips any with less
than 0.9 m of clearance, and drives up to 1 m toward the best one. Stay nearby: it's a
simple explorer, not a navigation stack.

### Go to a point

```bash
./ros python3 /work/tools/goto.py 1.5 -0.5      # map coordinates, meters
```

It drives in a straight line in checked steps, re-aiming between them, and stops if the
way is blocked. It doesn't plan around obstacles.

### See where you are

```bash
./ros python3 /work/tools/map_snapshot.py
```

This writes `maps/snapshot.png` (map plus robot pose and a 1 m scale bar) and prints the
distance to the nearest mapped obstacle in 8 directions.

## Save a map

```bash
./ros ros2 run nav2_map_server map_saver_cli -f /root/map/home --free 0.196 --occ 0.65
./ros ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /root/map/home}"
```

The first command writes `maps/home.pgm` and `maps/home.yaml`, a standard occupancy grid
for Nav2. The second writes `home.posegraph` and `home.data`, which slam_toolbox can load
to continue mapping later. `maps/` is git-ignored: a map is a floor plan of your home.

Yahboom's `save_map.launch.py` needs Cartographer, which isn't installed; use the
commands above instead.

## Check a map against a tape measure

Measure a few wall-to-wall spans in the room and compare them with the map. In the map,
take the occupied cells on the inside face of each wall and count the cells between their
centers (times the 0.05 m resolution). Counting only the free cells between the walls reads
about one cell (5 cm) short.

Our home map (2026-11-12, calibrated odometry, 8 m lidar range, one loop back to the start):

| | Tape | Map |
|---|---|---|
| Hallway width | 0.91 m | 0.90-0.95 m |
| Kitchen, wall to cabinet door | 2.26 m | 2.20-2.25 m |
| Whole house, living room wall to front door | 9.50 m | 9.60-9.65 m |

Within about 1%. The lidar over-reads range differences by about 3% (see
[calibration](07-calibration.md)); the map shows only part of that, probably because the
calibrated wheel odometry holds the scan matcher back. By the time the robot got back to
the start, odometry had drifted 17 cm and 2.8 deg, which slam_toolbox corrected.
