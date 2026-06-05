import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # get base sim launch for ros_gz_bridge yaml config file
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    upstream_yaml_path = os.path.join(sim_dir, 'config', 'tb4_bridge.yaml')

    # locate yaml patch
    my_package_name = 'state_estimation'
    my_package_dir = get_package_share_directory(my_package_name)
    patch_yaml_path = os.path.join(my_package_dir, 'config', 'tb4_bridge_patch.yaml')

    # read yaml files
    with open(upstream_yaml_path, 'r') as f:
        upstream_data = yaml.safe_load(f)
    with open(patch_yaml_path, 'r') as f:
        patch_data = yaml.safe_load(f)

    # purge and append as necessary
    purged_data = [entry for entry in upstream_data if entry.get('ros_topic_name') != 'cmd_vel']
    purged_data.append.extend(patch_data)

    # export to temp runtime folder
    generated_bridge_yaml = 'tmp/generated_tb4_bridge.yaml'
    with open(generated_bridge_yaml, 'w') as f:
        yaml.safe_dump(purged_data, f)

    # read main nav2 bringup file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb4_simulation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(my_package_dir, 'config', 'nav2_params.yaml'),
            'bridge_



