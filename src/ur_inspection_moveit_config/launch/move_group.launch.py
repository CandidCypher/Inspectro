"""MoveIt 2 move_group (+ optional RViz / Servo) for the UR + Zivid cell.

Mirrors ur_moveit_config/launch/ur_moveit.launch.py (Jazzy) but loads this package's SRDF and
configs, so the bracket and camera are part of the collision model. The URDF is not loaded here:
move_group and RViz pick it up from the /robot_description topic published by the UR control launch.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

UR_TYPES = [
    "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e", "ur8long",
    "ur15", "ur18", "ur20", "ur30", "ur10g-1750", "ur17g-1300", "ur18g-950",
]


def load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    declared_arguments = [
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "ur_type", default_value="ur30", choices=UR_TYPES, description="UR robot model."
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
    ]

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_inspection_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur_inspection.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Blocks until /robot_description is available, then move_group/RViz/Servo are started.
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_servo": load_yaml("ur_moveit_config", "config/ur_servo.yaml")},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            wait_robot_description,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_description,
                    on_exit=[move_group_node, rviz_node, servo_node],
                )
            ),
        ]
    )
