from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('aws_robomaker_small_warehouse_world'))
            / 'launch' / 'small_warehouse.launch.py',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': ParameterValue(Command([
                    FindExecutable(name='xacro'), ' ',
                    PathSubstitution(FindPackageShare('warehouse_amr')) / 'urdf' / 'warehouse_amr_ur5e.xacro',
                ]), value_type=str)},
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'warehouse_amr',
                       '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--param-file',
                       PathSubstitution(FindPackageShare('warehouse_amr')) / 'config' / 'amr_controllers.yaml'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--param-file',
                       PathSubstitution(FindPackageShare('warehouse_amr')) / 'config' / 'amr_controllers.yaml'],
            output='screen',
        ),
        # Relays: Nav2 expects plain /odom and /cmd_vel_smoothed-style final
        # output on generic names, but our controller publishes under its
        # own /diff_drive_controller/* namespace. These relays bridge that
        # gap automatically instead of needing manual background commands.
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            arguments=['/diff_drive_controller/odom', '/odom'],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel_smoothed', '/diff_drive_controller/cmd_vel'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--param-file',
                       PathSubstitution(FindPackageShare('warehouse_amr')) / 'config' / 'amr_controllers.yaml'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--param-file',
                       PathSubstitution(FindPackageShare('warehouse_amr')) / 'config' / 'amr_controllers.yaml'],
            output='screen',
        ),
    ])
