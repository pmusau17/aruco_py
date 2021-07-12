import os
import sys
import time
import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing
import pytest
import rclpy

from std_msgs.msg import String 

@pytest.mark.rostest
def generate_test_description():
    path_to_aruco_image_publisher_fixture = os.path.join(os.path.dirname(__file__), 'fixtures', 'aruco_image_publisher.py')
    path_to_test_data = os.path.join(os.path.dirname(__file__), 'data')
    print(path_to_test_data)
    path_to_image = os.path.join(path_to_test_data, 'marker23.png')

    return LaunchDescription([
        # Aruco image publisher
        Node(
            executable=sys.executable,
            arguments=[
                path_to_aruco_image_publisher_fixture,
                path_to_image
            ],
            output='screen'
        ),
        # DisparityNode
        Node(
            package='aruco_py',
            executable='detectVideo',
            name='detect_node',
            arguments=[
                '-t', 
                'DICT_6X6_250'
            ],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestArucoNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_aruco_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_message_received(self):
        # Expect the point cloud node to publish on '/points2' topic
        msgs_received = []
        self.node.create_subscription(
            String,
            'num_markers',
            lambda msg: msgs_received_rgb.append(msg),
            1
        )

        # Wait up to 60 seconds to receive message
        start_time = time.time()
        while (len(msgs_received) == 0 and (time.time() - start_time) < 60):
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            
        assert len(msgs_received) > 0
