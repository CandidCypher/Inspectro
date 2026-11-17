"""robot_state_publisher for the UR + Zivid description.

Intended as the ``description_launchfile`` for ``ur_robot_driver/ur_control.launch.py``.
It reuses ``ur_robot_driver/ur_rsp.launch.py`` and only swaps in this package's xacro, so every
UR argument (ur_type, use_mock_hardware, robot_ip, tool_*, ...) behaves exactly as upstream.

ur_control.launch.py sets these as launch configurations before including this file. They are
forwarded explicitly because launch only accepts required arguments of an included file when
they are passed to that include by name.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# (name, default used when this file is launched on its own; None = required)
FORWARDED_ARGS = [
    ("ur_type", None),
    ("robot_ip", "0.0.0.0"),
    ("tf_prefix", ""),
    ("use_mock_hardware", "false"),
    ("mock_sensor_commands", "false"),
    ("headless_mode", "false"),
    ("safety_limits", "true"),
    ("use_tool_communication", "false"),
    ("tool_parity", "0"),
    ("tool_baud_rate", "115200"),
    ("tool_stop_bits", "1"),
    ("tool_rx_idle_chars", "1.5"),
    ("tool_tx_idle_chars", "3.5"),
    ("tool_device_name", "/tmp/ttyUR"),
    ("tool_tcp_port", "54321"),
    ("tool_voltage", "0"),
]


def generate_launch_description():
    description_file = LaunchConfiguration(
        "description_file",
        default=PathJoinSubstitution(
            [FindPackageShare("ur_inspection_description"), "urdf", "ur_inspection.urdf.xacro"]
        ),
    )

    launch_arguments = {"description_file": description_file}
    for name, default in FORWARDED_ARGS:
        launch_arguments[name] = (
            LaunchConfiguration(name) if default is None else LaunchConfiguration(name, default=default)
        )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
