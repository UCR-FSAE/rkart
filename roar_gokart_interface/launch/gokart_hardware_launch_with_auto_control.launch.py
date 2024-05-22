"""_summary_description = Launch file to bring up the gokart hardware with manual control
"""

"""_summary_description = Launch file to bring up the gokart sensors.
"""

# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition  # 1

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"


def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_hardware_launches"))
    params_file = LaunchConfiguration('params_file')
    manual_control = LaunchConfiguration('manual_control', default="False")  
   
    """
    Sensor Bring up
    """
    hardware_launch_path: Path = (base_path
        / "launch"
        / "gokart_sensor_bringup.launch.py"
    )
    assert hardware_launch_path.exists(), f"[{hardware_launch_path}] does not exist"
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_path.as_posix())
    )

    """
    converter_node
    """
    converter_node = launch_ros.actions.Node(
        package="gokart_hardware_launches",
        executable="roar_gokart_converter",
        namespace="converter_node",
        output="screen",
        remappings=[("/converter_node/roar_control", "/roar/vehicle/control"),
                    ("/converter_node/gokart_control","/roar/gokart/control"),
                    ("/converter_node/roar_status", "/roar/vehicle/status"),
                    ("/converter_node/gokart_status", "/roar/gokart/status"),]
    )

    """
    gps -> odom converter
    """
    gps_interface_path: Path = Path(get_package_share_directory('roar_pointone_nav_interface')) / 'launch' / 'interface.launch.py'
    assert gps_interface_path.exists(), f"[{gps_interface_path}] does not exist"
    gps_interface_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gps_interface_path.as_posix()),
    launch_arguments={
            "params_file": params_file
        }.items())

    ld = launch.LaunchDescription()
    ld.add_action(converter_node)
    ld.add_action(hardware_launch)
    ld.add_action(gps_interface_launch)
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for all launched nodes'))
    return ld


if __name__ == "__main__":
    generate_launch_description()
