import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    my_package = get_package_share_directory('state_estimation')

    my_nav2_params          = os.path.join(my_package, 'config', 'nav2_params.yaml')
    my_ros_gz_bridge_params = os.path.join(my_package, 'config', 'tb4_bridge_patch.yaml')

    # ── launch arguments ────────────────────────────────────────────────────
    # use_sim_time is re-declared here so it can be forwarded to both the
    # inner launch file and the test_trajectory node from a single CLI arg.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock if true',
    )

    # tb4_object_index selects which entry in the gz PoseArray belongs to the
    # TurtleBot4 (depends on the order objects were spawned in the world).
    declare_tb4_index = DeclareLaunchArgument(
        'tb4_object_index',
        default_value='1',
        description="TurtleBot4's index within the /tb4_dynamic_pose PoseArray",
    )

    # test_trajectory_version selects which compiled executable to run.
    # The three files (test_trajectory_1/2/3/4/5.cpp) are never launched together.
    declare_test_traj_version = DeclareLaunchArgument(
        'test_trajectory_version',
        default_value='1',
        choices=['1', '2', '3', '4', '5'],    # 1: no set trajectory 2: short ThroughPose 3: long ThroughPose wp 4: short ToPose 5: longToPose
        description='Which test_trajectory executable to run (1, 2, 3, 4, or 5)',
    )

    # Milliseconds to wait after publishing /initialpose before sending the
    # NavigateThroughPoses goal.  action_server_is_ready() is true as soon as
    # the ROS interface exists, but bt_navigator (a lifecycle node) may still
    # be INACTIVE while costmaps load.  Increase if goals are still rejected.
    declare_nav_goal_delay = DeclareLaunchArgument(
        'nav_goal_delay_ms',
        default_value='5000',
        description='Delay (ms) between /initialpose and the first nav goal',
    )

    use_sim_time          = LaunchConfiguration('use_sim_time')
    tb4_object_index      = LaunchConfiguration('tb4_object_index')
    test_traj_version     = LaunchConfiguration('test_trajectory_version')
    nav_goal_delay_ms     = LaunchConfiguration('nav_goal_delay_ms')

    # ── 1. full simulation bringup ───────────────────────────────────────────
    # Launches: gz sim server, gz-ros bridge, robot_state_publisher,
    #           robot spawn, Nav2 bringup, RViz
    launch_tb4_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_package, 'launch', 'tb4_simulation_patched.launch.py')
        ),
        launch_arguments={
            'params_file':  my_nav2_params,
            'bridge_config': my_ros_gz_bridge_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── 2. test trajectory node ──────────────────────────────────────────────
    # Responsibilities:
    #   • accumulates ground-truth path from gz bridge (/tb4_dynamic_pose)
    #   • injects /initialpose to seed AMCL localisation
    #   • sends a NavigateThroughPoses goal once Nav2 is up
    #
    # Race-condition handling (two-layer):
    #
    #   Layer A — launch-level TimerAction delay (period below):
    #     Gives DDS / ROS graph enough time to propagate topic and service
    #     advertisements before the node begins its readiness checks.
    #     This avoids false-negative get_subscription_count() == 0 results
    #     that can occur in the very first seconds after ros2 launch.
    #
    #   Layer B — in-node readiness polling (checkReadiness(), 1 s wall timer):
    #     The node polls three non-blocking gates before doing anything:
    #       [AMCL]   init_publisher_->get_subscription_count() > 0
    #                  → AMCL has subscribed to /initialpose
    #       [bridge] tb4_gt_pose_subscriber_->get_publisher_count() > 0
    #                  → gz-ros bridge is publishing /tb4_dynamic_pose
    #       [nav2]   action_client_->action_server_is_ready()
    #                  → NavigateThroughPoses action server is available
    #     All three must pass before /initialpose is published or a nav goal
    #     is sent.  Status is logged every 3 s so progress is visible.
    #
    # Note: all timers inside the node use create_wall_timer(), so they
    # fire on real wall-clock time and are unaffected by sim time not yet
    # flowing (i.e. before the first /clock message arrives from Gazebo).
    
    kf_node = Node(
        package='state_estimation',
        executable='kf',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='kf_node',
        output='screen'
    )

    kf_quat_error_node = Node(
        package='state_estimation',
        executable='kf_wrong_quat',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='kf_quat_error_node',
        output='screen'
    )
    
    ekfo_node = Node(
        package='state_estimation',
        executable='ekf_odom',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],    
        name='ekf_sensors_node',
        output='screen'
    )

    ekfg_node = Node(
        package='state_estimation',
        executable='ekf_gated',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='ekf_gated_landmarks_node',
        output='screen'
    )

    ekfo_quat_error_node = Node(
        package='state_estimation',
        executable='ekf_odom_wrong_quat',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='ekfo_quat_error_node',
        output='screen'
    )
    
    ekfug_node = Node(
        package='state_estimation',
        executable='ekf_ungated',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='ekf_ungated_landmarks_node',
        output='screen'
    )

    pf_node = Node(
        package='state_estimation',
        executable='pf',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='pf_node',
        output='screen'
    )

    pf_quat_error_node = Node(
        package='state_estimation',
        executable='pf_wrong_quat',
        parameters=[os.path.join(my_package, 'config', 'balanced.yaml')],
        name='pf_quat_error_node',
        output='screen'
    )

    test_trajectory_node = Node(
        package='state_estimation',
        executable=PythonExpression(["'test_trajectory_' + '", test_traj_version, "'"]),
        name=PythonExpression(["'test_trajectory_' + '", test_traj_version, "'"]),
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'tb4_object_index':  tb4_object_index,
            'nav_goal_delay_ms': nav_goal_delay_ms,
        }],
    )

    # 5 s wall-clock delay: enough for DDS advertisements to propagate after
    # all simulation processes have been launched.  The in-node polling
    # handles the remaining wait for AMCL / Nav2 / bridge readiness.
    delayed_test_node = TimerAction(
        period=5.0,
        actions=[test_trajectory_node],
    )

    # ── launch description ───────────────────────────────────────────────────
    return LaunchDescription([
        declare_use_sim_time,
        declare_tb4_index,
        declare_test_traj_version,
        declare_nav_goal_delay,
        kf_node,
        kf_quat_error_node,
        ekfo_node,                
        ekfo_quat_error_node,
        ekfug_node,
        ekfg_node,    
        pf_node,
        pf_quat_error_node,
        launch_tb4_simulation,
        delayed_test_node,
    ])
