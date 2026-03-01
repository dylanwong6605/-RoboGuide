import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_nav = get_package_share_directory('amr_navigation')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time')

    # World file
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'test_obstacles.world')
    
    # Start Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )
    
    # Robot description
    urdf_file = os.path.join(pkg_amr_gazebo, 'urdf', 'hospital_amr.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hospital_amr'],
        output='screen'
    )

    # SLAM Toolbox
    slam_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ]
    )

    # Nav2 navigation stack
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    # YOLO perception nodes (multi-process sharding)
    yolo_node_0 = Node(
        package='amr_perception',
        executable='yolo_perception',
        name='yolo_detector_0',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolo26n.pt',
            'image_topic': '/camera/image_raw',
            'detections_topic': '/perception/detections',
            'annotated_image_topic': '/perception/annotated_image',
            'conf_threshold': 0.25,
            'input_size': 640,
            'use_fp16': True,
            'worker_count': 2,
            'worker_index': 0,
            'publish_annotated': True,
        }]
    )

    yolo_node_1 = Node(
        package='amr_perception',
        executable='yolo_perception',
        name='yolo_detector_1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': 'yolo26n.pt',
            'image_topic': '/camera/image_raw',
            'detections_topic': '/perception/detections',
            'annotated_image_topic': '/perception/annotated_image',
            'conf_threshold': 0.25,
            'input_size': 640,
            'use_fp16': True,
            'worker_count': 2,
            'worker_index': 1,
            'publish_annotated': False,
        }]
    )


    
    # YOLO-based reactive controller: listens to /perception/detections and
    # publishes zero velocity on /cmd_vel when a person is detected.
    yolo_reactive_controller = Node(
        package='amr_perception',
        executable='yolo_reactive_controller',
        name='yolo_reactive_controller',
        output='screen',
        parameters=[{
            'detections_topic': '/perception/detections',
            'cmd_vel_topic': '/cmd_vel',
            'person_class_id': 'person',
            'min_score': 0.5,
            'hold_time': 0.5,
        }]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,

        # Simulation
        gazebo,
        robot_state_publisher,
        spawn_entity,

        # Navigation
        slam_node,
        nav2_bringup,

        # Perception
        yolo_node_0,
        yolo_node_1,


        # Reactive controller
        yolo_reactive_controller,
    ])
