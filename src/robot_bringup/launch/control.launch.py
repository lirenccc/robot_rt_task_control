from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rt_loop_hz = LaunchConfiguration('rt_loop_hz')
    use_fifo_scheduler = LaunchConfiguration('use_fifo_scheduler')
    rt_priority = LaunchConfiguration('rt_priority')
    hardware_bus_plugin = LaunchConfiguration('hardware_bus_plugin')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_description'),
            'urdf',
            'mobile_manipulator.urdf.xacro'
        ]),
        ' ',
        'rt_loop_hz:=', rt_loop_hz,
        ' ',
        'use_fifo_scheduler:=', use_fifo_scheduler,
        ' ',
        'rt_priority:=', rt_priority,
        ' ',
        'hardware_bus_plugin:=', hardware_bus_plugin,
    ])

    robot_description = {'robot_description': robot_description_content}

    controllers_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'controllers.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rt_loop_hz', default_value='1000.0'),
        DeclareLaunchArgument('use_fifo_scheduler', default_value='false'),
        DeclareLaunchArgument('rt_priority', default_value='70'),
        DeclareLaunchArgument(
            'hardware_bus_plugin',
            default_value='robot_hardware_plugins/MockHardwareBus'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_file],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['base_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
