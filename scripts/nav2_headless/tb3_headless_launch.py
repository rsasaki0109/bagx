"""Headless Nav2 TB3 launch with explicit Gazebo headless rendering."""

from __future__ import annotations

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")

    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_simulator = LaunchConfiguration("use_simulator")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    pose = {
        "x": LaunchConfiguration("x_pose", default="-2.00"),
        "y": LaunchConfiguration("y_pose", default="-0.50"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    render_engine_server = LaunchConfiguration("render_engine_server")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    urdf = os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, "r", encoding="utf-8") as infp:
        robot_description = infp.read()

    world_sdf = tempfile.mktemp(prefix="nav2_headless_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )
    gazebo_server = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "--headless-rendering",
            "--render-engine-server",
            render_engine_server,
            "-r",
            "-s",
            world_sdf,
        ],
        output="screen",
        condition=IfCondition(use_simulator),
    )
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
        remappings=remappings,
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "robot_sdf": robot_sdf,
            "x_pose": pose["x"],
            "y_pose": pose["y"],
            "z_pose": pose["z"],
            "roll": pose["R"],
            "pitch": pose["P"],
            "yaw": pose["Y"],
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_namespace",
            default_value="false",
            description="Whether to apply a namespace to the navigation stack",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam", default_value="False", description="Whether run a SLAM"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(bringup_dir, "maps", "tb3_sandbox.yaml"),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
            description="Full path to the ROS2 parameters file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
            description="Whether to use composed bringup",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn crashed nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
            description="Full path to the RVIZ config file to use",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_simulator",
            default_value="True",
            description="Whether to start the simulator",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="True",
            description="Whether to start the robot state publisher",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="False", description="Whether to start RVIZ"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="True",
            description="Whether to run the headless simulator path",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(sim_dir, "worlds", "tb3_sandbox.sdf.xacro"),
            description="Full path to the world xacro file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="turtlebot3_waffle",
            description="Name of the robot",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_sdf",
            default_value=os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro"),
            description="Full path to the robot sdf file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "render_engine_server",
            default_value="ogre2",
            description="Gazebo render engine to use on the server",
        )
    )

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(world_sdf_xacro)
    ld.add_action(gazebo_server)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gz_robot)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(gazebo_client)
    return ld
