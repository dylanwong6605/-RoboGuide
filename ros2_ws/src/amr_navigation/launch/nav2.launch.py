import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ── Package directories ──────────────────────────────────────────────────
    pkg_nav   = get_package_share_directory('amr_navigation')
    pkg_nav2  = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Run SLAM Toolbox alongside Nav2 (set false when using a pre-built map)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to a saved map .yaml (only used when use_slam=false)'
    )

    # ── Substitutions ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam     = LaunchConfiguration('use_slam')
    map_yaml     = LaunchConfiguration('map')

    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_nav, 'config', 'slam_params.yaml')

    # ── SLAM Toolbox node ────────────────────────────────────────────────────
    slam_node = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Nav2 bringup (without map_server when doing SLAM) ───────────────────
    # We use the nav2_bringup navigation_launch.py which starts:
    #   controller_server, planner_server, behavior_server,
    #   bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            # When use_slam=true the map comes from slam_toolbox, not map_server
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_slam_arg,
        map_yaml_arg,
        slam_node,
        nav2_bringup,
    ])
