# Copyright 2026
# Launch integration test: in-process mock task stack.

import os
import time
import unittest

# Avoid writing under ~/.ros when the environment is read-only (CI sandboxes).
os.environ.setdefault('ROS_HOME', '/tmp/robot_testkit_ros_home')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/robot_testkit_ros_log')
os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from robot_interfaces.action import Task


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('robot_bringup')
    launch_path = os.path.join(bringup, 'launch', 'task_stack_inprocess_mock.launch.py')
    return (
        launch.LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestTaskStackInprocess(unittest.TestCase):

    def test_execute_succeeds(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('test_task_stack_client')
        client = ActionClient(node, Task, '/task/execute')
        self.assertTrue(client.wait_for_server(timeout_sec=20.0))

        goal = Task.Goal()
        goal.instruction = 'go to table and pick red cup'
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        deadline = time.time() + 30.0
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertTrue(result_future.done())
        result = result_future.result().result
        self.assertTrue(result.success, result.message)

        node.destroy_node()
        rclpy.shutdown()
