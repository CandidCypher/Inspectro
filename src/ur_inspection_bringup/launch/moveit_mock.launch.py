"""Milestone 1: UR (default UR30) + Zivid MR60 with MoveIt 2 on ros2_control mock hardware.

    ros2 launch ur_inspection_bringup moveit_mock.launch.py [ur_type:=ur30] [launch_rviz:=true]

Starts:
  * ur_robot_driver/ur_control.launch.py with use_mock_hardware:=true, using this project's
    UR + Zivid description (via description_launchfile) and joint_trajectory_controller
  * ur_inspection_moveit_config/move_group.launch.py (move_group + RViz MotionPlanning)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur30", description="UR robot model."),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz with MoveIt?"
        ),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            # Required by ur_control.launch.py but unused with mock hardware.
            "robot_ip": "0.0.0.0",
            "use_mock_hardware": "true",
            "mock_sensor_commands": "false",
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": "false",
            "description_launchfile": PathJoinSubstitution(
                [FindPackageShare("ur_inspection_description"), "launch", "rsp.launch.py"]
            ),
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_inspection_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": launch_rviz}.items(),
    )

    # Scope the UR include so its launch_rviz:=false (and other args) do not leak into the
    # shared launch context and override this file's launch_rviz for the MoveIt include.
    return LaunchDescription(
        declared_arguments
        + [GroupAction([ur_control], scoped=True, forwarding=True), move_group]
    )
