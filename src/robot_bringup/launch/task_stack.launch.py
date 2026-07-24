from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    runtime_name = LaunchConfiguration('runtime_config').perform(context)
    profile = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'profile.yaml'])
    runtime = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', runtime_name])

    nodes = []
    # Mock Action servers only for default ros_action runtime; Nav2 / MoveIt replace them.
    if runtime_name in ('runtime.yaml',):
        nodes.extend([
            Node(
                package='robot_navigation_adapters',
                executable='mock_navigation_server',
                name='mock_navigation_server',
                output='screen',
            ),
            Node(
                package='robot_manipulation_adapters',
                executable='mock_manipulation_server',
                name='mock_manipulation_server',
                output='screen',
            ),
        ])

    nodes.append(
        Node(
            package='robot_runtime',
            executable='task_orchestrator_node',
            name='task_orchestrator_node',
            output='screen',
            parameters=[{
                'profile_file': profile,
                'runtime_file': runtime,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_config',
            default_value='runtime.yaml',
            description=(
                'Runtime YAML under robot_bringup/config '
                '(e.g. runtime.yaml, runtime_nav2.yaml, runtime_moveit.yaml, runtime_mock.yaml)'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
