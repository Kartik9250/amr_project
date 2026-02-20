import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config = os.path.join(pkg, 'rviz', 'config.rviz')

    description_pkg = get_package_share_directory('mr_robot_description')

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'mr_robot.xacro'
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            {'publish_frequency': 50.0},
        ],
        output='screen'
    )

    world_file = os.path.join(pkg, 'worlds', 'amr_world.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',         
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ],
    output='screen'
    )

    # Separate bridge for tf_static with correct QoS
    bridge_tf_static = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        ros_arguments=['--ros-args', '--remap', '/tf_static:=/tf_static'],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg, 'maps', 'map.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    initial_pose = TimerAction(
    period=8.0,  # give Nav2 time to fully start
    actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.07]}}'
            ],
            output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        bridge_tf_static,
        rviz,
        nav2,
        # initial_pose
    ])

