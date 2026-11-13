# 9. Navigating with Nav2

Once you have a saved map ([mapping](05-mapping.md)), Nav2 can drive the robot to any point
on it, planning around furniture by itself.

```bash
./up nav                     # Nav2 on maps/home.yaml; MAP=other.yaml ./up nav for another map
```

This starts the base, then Nav2 with [config/nav2.yaml](../config/nav2.yaml): AMCL for
localization on the saved map, the Navfn planner, and the Regulated Pure Pursuit controller
at 0.15 m/s. If you recreated the container, wait until the robot has reconnected (the
bridge log shows a new session) before sending goals; Nav2 needs `/odom`.

## Tell AMCL where the robot is

AMCL starts at map (0, 0, 0), where the mapping run started. If the robot is somewhere else,
or facing another way:

```bash
./ros python3 /work/tools/nav_goal.py init X Y YAW_DEG      # map meters, degrees
```

AMCL listens on `/initialpose` best-effort, so the tool waits for it to connect and repeats
the message for 3 seconds. You can also use Foxglove's pose-estimate tool.

## Send goals

```bash
./ros python3 /work/tools/nav_goal.py go 3.5 -0.45 0 --name den-door
```

It prints whether the goal succeeded, how long it took and how far the robot ended up from
the goal (by AMCL), and appends a row to `datasets/nav/goals.csv`. Watch the plan and the
costmaps in Foxglove (`/plan`, `/local_costmap/costmap`, `/global_costmap/costmap`).

## Repeated loops

```bash
./ros python3 /work/tools/nav_loop.py --laps 5
```

Drives a fixed loop of waypoints (edit `WAYPOINTS` for your home) and at each arrival
records AMCL's pose, a scan match against the saved map, and a scan match against the first
lap's arrival there (how repeatable the arrivals are). Rows go to `datasets/nav/loop_*.csv`.

## Results on our robot

Home map, loop from the kitchen through the main room and hallway to a den doorway, the far
end of the living room and back (about 15 m), 2026-11-12. 45 goals, all succeeded, 52-79 s
each.

| Settings | Distance from goal | Heading error |
|---|---|---|
| Nav2 AMCL defaults, 10 cm / 0.15 rad goal tolerance | 4-10 cm | up to 8 deg |
| Tighter AMCL sensor model, 5 cm / 0.08 rad tolerance | 1-4 cm | up to 8 deg |
| Same, final turn at 0.3 rad/s instead of 0.5 | 1-4 cm | within 3.6 deg |

Repeated arrivals at the same spot landed within 2-5 cm and a few degrees of each other.

What made the difference:

- **AMCL's sensor model.** Nav2's defaults (`sigma_hit` 0.2 m, half the returns treated as
  random) suit a noisy lidar. With `sigma_hit` 0.05, `z_hit` 0.9 and 180 beams, repeated
  arrivals got tighter.
- **Goal tolerance.** The robot stops as soon as it is inside it, so it ends up near the
  edge: with 10 cm, it stopped 6-9 cm short.
- **Coasting.** The robot keeps turning 3-5 deg after a turn command stops (see
  [calibration](07-calibration.md)). At 0.5 rad/s, Nav2 finished its final turn inside the
  tolerance and the robot coasted past it; at 0.3 rad/s it stays inside.

Measuring accuracy without an external reference is limited: matching a scan against the map
at the kitchen start, which looks down a narrow strip, gave answers spread over about 4 cm and
2 deg with equally good fits. AMCL and the map match agreed within that. A first version
matched against a single earlier scan instead and showed a consistent 3 deg "AMCL error" that
was really the matcher trading heading for sideways shift. For centimeter ground truth, use
the tape and overhead camera or ArUco markers.

## Checking a new ROS release against a recorded run

`tools/replay_amcl.py` plays a recorded Nav2 run (`datasets/raw/<run>`, see
[recording](08-recording.md)) into a fresh AMCL and compares its pose, once per second, with
the pose AMCL produced live on the robot. With `--nav` it starts the whole Nav2 bringup
instead and also reports whether every Nav2 node reached the active state. The robot isn't
needed; this is how we checked ROS 2 Jazzy before running it on the robot.

Run it in a throwaway container with its own domain ID, so it can't talk to the live stack:

```bash
d=jazzy   # or humble
docker run --rm -e ROS_DOMAIN_ID=92 \
  -v $PWD/maps:/root/map -v $PWD/tools:/work/tools -v $PWD/config:/work/config:ro \
  -v $PWD/datasets:/root/datasets lidarbot-ros:$d \
  python3 /work/tools/replay_amcl.py /root/datasets/raw/<run> /root/datasets/nav/replay_$d.csv --rate 2 --nav
```

It uses `config/nav2_<distro>.yaml` if there is one, otherwise `config/nav2.yaml`.

Results for the 7-minute tuned loop run of 2026-11-12 (difference from the live AMCL pose, first
10 s skipped while AMCL settles):

| Replay | Position: median / 95% / max | Heading: median / max |
|---|---|---|
| Humble, AMCL only | 1.1 / 2.6 / 3.4 cm | 0.36 / 2.13 deg |
| Jazzy, AMCL only | 1.2 / 2.5 / 3.6 cm | 0.34 / 2.13 deg |
| Humble, full Nav2 | 1.0 / 2.1 / 3.2 cm | 0.33 / 2.55 deg |
| Jazzy, full Nav2 | 1.1 / 2.2 / 3.5 cm | 0.39 / 1.96 deg |

The two releases match each other as closely as Humble matches itself. The replay isn't zero
against the live run on either: AMCL's particle filter is random, and the bag plays at 2x.

What Jazzy's Nav2 needed beyond the Humble settings (`config/nav2_jazzy.yaml`):

- **Plugin names with `::`.** `nav2_navfn_planner::NavfnPlanner`; the old
  `nav2_navfn_planner/NavfnPlanner` form stops the planner from starting.
- **collision_monitor.** Jazzy's bringup puts it between the velocity smoother and the robot
  (`cmd_vel_smoothed` -> `cmd_vel`). Ours slows the robot so its footprint never reaches a
  lidar point within 1.2 s.
- **docking_server.** Also started by Jazzy's bringup. One simple charging dock plugin, no
  docks defined yet.
- **Startup order in the replay.** Jazzy's costmaps fail if the odometry transform is
  missing when they start. On the robot it is always there; in the replay the bag starts
  first and AMCL is given the live starting pose.
