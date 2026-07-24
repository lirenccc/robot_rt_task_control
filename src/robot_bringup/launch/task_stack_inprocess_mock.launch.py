from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'profile.yaml'])
    runtime = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'runtime_mock.yaml'])

    return LaunchDescription([
        Node(
            package='robot_runtime',
            executable='task_orchestrator_node',
            name='task_orchestrator_node',
            output='screen',
            parameters=[{
                'profile_file': profile,
                'runtime_file': runtime,
            }],
        ),
    ])
