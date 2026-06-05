import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # base package
    my_package = get_package_share_directory('state_estimation')

    # patched params
    my_nav2_params = os.path.join(my_package, 'config', 'nav2_params.yaml')
    my_ros_gz_bridge_params = os.path.join(my_package, 'config', 'tb4_bridge_patch.yaml')

    # tb4_sim bringup
    launch_tb4_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_package, 'launch', 'tb4_simulation_patched.launch.py')
        ),
        launch_arguments={
            'params_file': my_nav2_params,
            'bridge_config': my_ros_gz_bridge_params,
        }.items(),
    )

    return LaunchDescription([launch_tb4_simulation])

