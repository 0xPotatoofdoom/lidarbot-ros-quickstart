"""The rover's ROS stack on the Orange Pi (ROS 2 Jazzy, native): replaces the Mac's Docker stack.

Starts the micro-ROS agent, the robot model (static TFs; pan/tilt joints from ptz_joint_states.py), the odometry
calibrator, the EKF (/odom), the battery monitor and foxglove_bridge (ws://<pi>:8765), plus the
lean Nav2 (nav_pi.launch.py) with nav:=true. Run from the repo copy on the Pi, domain 77:

    ROS_DOMAIN_ID=77 ros2 launch onboard/launch/rover_pi.launch.py urdf:=$HOME/lidarbot/private/robot.urdf nav:=true

Agent transport: agent_transport:=udp4 agent_port:=8090 when the rover sends straight to the Pi,
tcp4 / 8888 behind tools/udp_tcp_bridge.py, serial once micro-ROS runs over the USB cable.
The URDF is generated from Yahboom's robot_description (BSL, so not in this repo): see
docs/10-onboard-computer.md.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[2]          # the repo copy (~/lidarbot on the Pi)
CONFIG = ROOT / "config"
TOOLS = ROOT / "tools"
DATA = Path(os.environ.get("LIDARBOT_DATA", str(Path.home() / "lidarbot" / "data")))
ENV = {"LIDARBOT_CONFIG": str(CONFIG), "LIDARBOT_DATA": str(DATA)}


def python_node(script: Path, *args: str) -> ExecuteProcess:
    return ExecuteProcess(cmd=["python3", "-u", str(script), "--ros-args", *args],
                          additional_env=ENV, output="screen", name=script.stem)


def transport_is(value: str, equal: bool = True) -> IfCondition:
    op = "==" if equal else "!="
    return IfCondition(PythonExpression(["'", LaunchConfiguration("agent_transport"), f"' {op} '{value}'"]))


def generate_launch_description() -> LaunchDescription:
    urdf = LaunchConfiguration("urdf")
    return LaunchDescription([
        DeclareLaunchArgument("urdf", description="robot URDF generated from Yahboom's robot_description"),
        DeclareLaunchArgument("agent_transport", default_value="udp4", description="udp4 | tcp4 | serial"),
        DeclareLaunchArgument("agent_port", default_value="8090", description="UDP/TCP port"),
        DeclareLaunchArgument("agent_dev", default_value="/dev/ttyUSB0", description="serial device"),
        DeclareLaunchArgument("agent_baud", default_value="921600"),
        DeclareLaunchArgument("nav", default_value="false", description="start Nav2 too"),
        DeclareLaunchArgument("map", default_value=str(ROOT / "maps" / "home.yaml")),

        # The agent binary itself, not `ros2 run` (which didn't pass on SIGINT: an orphaned agent
        # kept the port and the next start failed with "bind error").
        Node(package="micro_ros_agent", executable="micro_ros_agent", name="micro_ros_agent",
             condition=transport_is("serial"),
             arguments=["serial", "--dev", LaunchConfiguration("agent_dev"), "-b", LaunchConfiguration("agent_baud")]),
        Node(package="micro_ros_agent", executable="micro_ros_agent", name="micro_ros_agent",
             condition=transport_is("serial", equal=False),
             arguments=[LaunchConfiguration("agent_transport"), "--port", LaunchConfiguration("agent_port")]),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(Command(["cat ", urdf]), value_type=str)}]),
        python_node(ROOT / "onboard" / "ptz_joint_states.py"),
        python_node(TOOLS / "odom_calibrator.py", "--params-file", str(CONFIG / "calibration.yaml")),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[str(CONFIG / "ekf.yaml")], remappings=[("/odometry/filtered", "/odom")]),
        python_node(TOOLS / "battery_monitor.py"),
        Node(package="foxglove_bridge", executable="foxglove_bridge", parameters=[{"port": 8765}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "onboard" / "launch" / "nav_pi.launch.py")),
            launch_arguments={"map": LaunchConfiguration("map"),
                              "params_file": str(CONFIG / "nav2_jazzy.yaml")}.items(),
            condition=IfCondition(LaunchConfiguration("nav"))),
    ])

