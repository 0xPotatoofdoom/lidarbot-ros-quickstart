#!/usr/bin/env python3
"""Send Nav2 goals on a saved map and measure how close the robot gets (./up nav first).

    ./ros python3 /work/tools/nav_goal.py init X Y YAW_DEG        # tell AMCL where the robot is
    ./ros python3 /work/tools/nav_goal.py go X Y YAW_DEG [--name den-door]

Map coordinates in meters, yaw in degrees (0 = map +x). After each goal it waits for the robot
to coast to a stop, reads its pose from AMCL (map -> base_footprint) and prints the distance
and heading error. Every goal is appended to /root/datasets/nav/goals.csv.
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

LOG = Path("/root/datasets/nav/goals.csv")
SETTLE_S = 1.5          # coasting after the last command
INIT_COV_XY = 0.05 ** 2
INIT_COV_YAW = math.radians(3.0) ** 2


@dataclass(frozen=True)
class Arrival:
    dist_m: float
    dyaw_deg: float


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def arrival_error(goal: tuple[float, float, float], pose: tuple[float, float, float]) -> Arrival:
    dyaw = math.atan2(math.sin(pose[2] - goal[2]), math.cos(pose[2] - goal[2]))
    return Arrival(math.hypot(pose[0] - goal[0], pose[1] - goal[1]), math.degrees(dyaw))


def log_row(row: dict) -> None:
    LOG.parent.mkdir(parents=True, exist_ok=True)
    new = not LOG.exists()
    with LOG.open("a") as f:
        if new:
            f.write(",".join(row) + "\n")
        f.write(",".join(str(v) for v in row.values()) + "\n")


# ---------------------------------------------------------------- robot side

def map_pose(node, tf, wait: float = 3.0) -> tuple[float, float, float] | None:
    import rclpy
    from tf2_ros import TransformException
    end = time.monotonic() + wait
    while time.monotonic() < end:
        try:
            t = tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            return t.transform.translation.x, t.transform.translation.y, quat_to_yaw(q.x, q.y, q.z, q.w)
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.1)
    return None


def set_initial_pose(node, x: float, y: float, yaw: float) -> None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    q = yaw_to_quat(yaw)
    o = msg.pose.pose.orientation
    o.x, o.y, o.z, o.w = q
    cov = [0.0] * 36
    cov[0] = cov[7] = INIT_COV_XY
    cov[35] = INIT_COV_YAW
    msg.pose.covariance = cov
    end = time.monotonic() + 10.0  # wait for AMCL to connect, or the message is lost
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        raise SystemExit("nothing subscribes to /initialpose; is ./up nav running?")
    # AMCL subscribes best-effort, so early messages can be lost while discovery settles.
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        stop = time.monotonic() + 0.5
        while time.monotonic() < stop:
            rclpy.spin_once(node, timeout_sec=0.05)


def nav_client(node):
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        raise SystemExit("navigate_to_pose isn't available; is ./up nav running?")
    return client


def drive_goal(node, tf, client, x: float, y: float, yaw: float
               ) -> tuple[str, float, tuple[float, float, float] | None]:
    """Send one goal and wait for it: (status, seconds, settled map pose or None)."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    o = goal.pose.pose.orientation
    o.x, o.y, o.z, o.w = yaw_to_quat(yaw)
    t0 = time.monotonic()
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        return "rejected", 0.0, map_pose(node, tf)
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res)
    elapsed = time.monotonic() - t0
    status = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
              GoalStatus.STATUS_CANCELED: "canceled"}.get(res.result().status, str(res.result().status))
    end = time.monotonic() + SETTLE_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return status, elapsed, map_pose(node, tf)


def navigate(node, tf, x: float, y: float, yaw: float, name: str) -> None:
    client = nav_client(node)
    start_pose = map_pose(node, tf)
    status, elapsed, pose = drive_goal(node, tf, client, x, y, yaw)
    if pose is None:
        print(f"{name}: {status} in {elapsed:.0f} s, but no map pose to measure")
        return
    err = arrival_error((x, y, yaw), pose)
    print(f"{name}: {status} in {elapsed:.0f} s, at {pose[0]:+.3f}, {pose[1]:+.3f}, "
          f"{math.degrees(pose[2]):+.1f} deg; off by {err.dist_m * 100:.1f} cm, {err.dyaw_deg:+.1f} deg")
    log_row({"unix_time": f"{time.time():.1f}", "name": name, "goal_x": x, "goal_y": y,
             "goal_yaw_deg": round(math.degrees(yaw), 1), "status": status, "seconds": round(elapsed, 1),
             "start_x": round(start_pose[0], 3) if start_pose else "",
             "start_y": round(start_pose[1], 3) if start_pose else "",
             "x": round(pose[0], 3), "y": round(pose[1], 3), "yaw_deg": round(math.degrees(pose[2]), 1),
             "error_cm": round(err.dist_m * 100, 1), "yaw_error_deg": round(err.dyaw_deg, 1)})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["init", "go"])
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("yaw_deg", type=float)
    ap.add_argument("--name", default="goal")
    args = ap.parse_args()
    import rclpy
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node("nav_goal")
    tf = Buffer()
    TransformListener(tf, node)
    try:
        yaw = math.radians(args.yaw_deg)
        if args.cmd == "init":
            set_initial_pose(node, args.x, args.y, yaw)
            time.sleep(1.0)
            pose = map_pose(node, tf)
            print("initial pose sent; AMCL now says", "nothing yet" if pose is None else
                  f"{pose[0]:+.3f}, {pose[1]:+.3f}, {math.degrees(pose[2]):+.1f} deg")
        else:
            navigate(node, tf, args.x, args.y, yaw, args.name)
    except KeyboardInterrupt:
        print("interrupted; Nav2 keeps its goal until canceled (or ./up base)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
