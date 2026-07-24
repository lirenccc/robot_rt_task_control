from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'launch', 'control.launch.py'])
    task = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'launch', 'task_stack.launch.py'])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(task)),
    ])
