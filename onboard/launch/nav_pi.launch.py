"""Lean Nav2 for the Orange Pi (ROS 2 Jazzy): localization and navigation in one process.

Jazzy's bringup_launch.py also starts the smoother, route, waypoint follower and docking servers,
which our default behaviour trees don't use; on the Pi's 4 x A53 every extra server costs CPU
even when idle. Velocity chain as in Jazzy's bringup:
controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel.

    ros2 launch onboard/launch/nav_pi.launch.py map:=maps/home.yaml params_file:=config/nav2_jazzy.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

NODES = [  # (package, plugin, name, extra remappings), in lifecycle start order
    ("nav2_map_server", "nav2_map_server::MapServer", "map_server", []),
    ("nav2_amcl", "nav2_amcl::AmclNode", "amcl", []),
    ("nav2_controller", "nav2_controller::ControllerServer", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_planner", "nav2_planner::PlannerServer", "planner_server", []),
    ("nav2_behaviors", "behavior_server::BehaviorServer", "behavior_server", [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_velocity_smoother", "nav2_velocity_smoother::VelocitySmoother", "velocity_smoother",
     [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_collision_monitor", "nav2_collision_monitor::CollisionMonitor", "collision_monitor", []),
    ("nav2_bt_navigator", "nav2_bt_navigator::BtNavigator", "bt_navigator", []),
]


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = {"use_sim_time": sim_time}
    nodes = [
        ComposableNode(package=pkg, plugin=plugin, name=name,
                       parameters=[params, common] + ([{"yaml_filename": LaunchConfiguration("map")}]
                                                      if name == "map_server" else []),
                       remappings=remaps)
        for pkg, plugin, name, remaps in NODES
    ]
    nodes.append(ComposableNode(
        package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
        name="lifecycle_manager",
        parameters=[{"use_sim_time": sim_time, "node_names": [n[2] for n in NODES],
                     "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool)}]))
    return LaunchDescription([
        DeclareLaunchArgument("map", description="map yaml"),
        DeclareLaunchArgument("params_file", description="Nav2 parameters (config/nav2_jazzy.yaml)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        ComposableNodeContainer(name="nav2_container", namespace="", package="rclcpp_components",
                                executable="component_container_isolated",
                                composable_node_descriptions=nodes, output="screen"),
    ])
