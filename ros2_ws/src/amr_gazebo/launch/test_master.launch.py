import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ── Package directories ──────────────────────────────────────────────
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav = get_package_share_directory('amr_navigation')
    
    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation clock'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_obstacles',
        description='World file name (without .world extension)'
    )
    
    # ── Substitutions ────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    
    # Build world file path
    world_file = [pkg_gazebo, '/worlds/', world_name, '.world']
    
    # ── 1. GAZEBO SIMULATION ─────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )
    
    # ── 2. ROBOT DESCRIPTION & STATE PUBLISHER ──────────────────────────
    urdf_file = os.path.join(pkg_gazebo, 'urdf', 'hospital_amr.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ── 3. SPAWN ROBOT IN GAZEBO ────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hospital_amr'],
        output='screen'
    )
    
    # ── 4. SLAM TOOLBOX ──────────────────────────────────────────────────
    slam_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')
    
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ── 5. NAV2 NAVIGATION STACK ────────────────────────────────────────
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items()
    )
    
    # ── 6. YOLO PERCEPTION NODE ─────────────────────────────────────────
    yolo_node = Node(
        package='amr_perception',
        executable='yolo_perception',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolo26n.pt',
            'image_topic': '/camera/image_raw',
            'detections_topic': '/perception/detections',
            'annotated_image_topic': '/perception/annotated_image',
            'conf_threshold': 0.25
        }]
    )
    
    # ── LAUNCH DESCRIPTION ───────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_arg,
        
        # Simulation
        gazebo,
        robot_state_publisher,
        spawn_entity,
        
        # Navigation
        slam_node,
        nav2_bringup,
        
        # Perception
        yolo_node,
    ])
