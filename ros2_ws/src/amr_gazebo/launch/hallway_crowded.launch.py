import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	pkg_gazebo = get_package_share_directory('amr_gazebo')
	pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
	pkg_nav = get_package_share_directory('amr_navigation')

	use_sim_time_arg = DeclareLaunchArgument(
		'use_sim_time',
		default_value='true',
		description='Use simulation clock'
	)

	world_arg = DeclareLaunchArgument(
		'world',
		default_value='hallway_crowded',
		description='World file name (without .world extension)'
	)

	use_sim_time = LaunchConfiguration('use_sim_time')
	world_name = LaunchConfiguration('world')
	world_file = [pkg_gazebo, '/worlds/', world_name, '.world']

	gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
		),
		launch_arguments={'world': world_file}.items(),
	)

	urdf_file = os.path.join(pkg_gazebo, 'urdf', 'hospital_amr.urdf')
	with open(urdf_file, 'r') as file:
		robot_desc = file.read()

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

	spawn_entity = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-topic', 'robot_description', '-entity', 'hospital_amr', '-x', '-12.0', '-y', '0.0', '-z', '0.5'],
		output='screen'
	)

	map_file = os.path.join(pkg_nav, 'maps', 'hallway_crowded.yaml')

	map_server = Node(
		package='nav2_map_server',
		executable='map_server',
		name='map_server',
		output='screen',
		parameters=[{
			'use_sim_time': use_sim_time,
			'yaml_filename': map_file,
		}]
	)

	nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

	amcl = Node(
		package='nav2_amcl',
		executable='amcl',
		name='amcl',
		output='screen',
		parameters=[nav2_params, {'use_sim_time': use_sim_time}]
	)

	set_initial_pose = TimerAction(
		period=15.0,
		actions=[
			ExecuteProcess(
				cmd=[
					'ros2', 'topic', 'pub', '--once', '/initialpose',
					'geometry_msgs/msg/PoseWithCovarianceStamped',
					'{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -12.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}'
				],
				output='screen'
			)
		]
	)

	localization_lifecycle = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager_localization',
		output='screen',
		parameters=[{
			'use_sim_time': use_sim_time,
			'autostart': True,
			'node_names': ['map_server', 'amcl'],
		}]
	)

	nav2_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
		),
		launch_arguments={
			'use_sim_time': 'true',
			'params_file': nav2_params,
		}.items(),
	)

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
			'min_person_bbox_height_px': 80.0,
			'hold_time': 0.5,
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
		use_sim_time_arg,
		world_arg,

		gazebo,
		robot_state_publisher,
		spawn_entity,

		map_server,
		amcl,
		set_initial_pose,
		localization_lifecycle,

		nav2_bringup,

		yolo_node_0,
		yolo_node_1,
		yolo_reactive_controller,

		rviz_node,
	])
