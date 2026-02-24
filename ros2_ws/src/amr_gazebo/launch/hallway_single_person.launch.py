import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav = get_package_share_directory('amr_navigation')
    
    models_path = os.path.join(pkg_gazebo, 'models')
    fuel_models_path = os.path.join(pkg_gazebo, 'fuel_models')
    
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + fuel_models_path
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation clock'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'hallway_single_person.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )
    
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
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hospital_amr', '-x', '0.0', '-y', '0.0', '-z', '0.6'],
        output='screen'
    )
    
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
    
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        set_gazebo_model_path,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        slam_node,
        nav2_bringup,
        yolo_node,
        rviz_node,
    ])