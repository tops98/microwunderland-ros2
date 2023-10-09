from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os


def generate_launch_description():
    path_to_param = os.path.join( get_package_share_directory("tracker_visualizer"),'config','parameters.yaml')
    params = dict()
    with open(path_to_param,'r') as file:
        params = yaml.load(file)

    return LaunchDescription([
        Node(
            package='object_detector',
            executable='object_detector',
            name='object_detector_node'
        ),
        Node(
            package='object_tracker',
            executable='object_tracker',
            name='tracker_node',
            parameters=[params["tracker_node"]['ros__parameters']]
        ),
        Node(
            package='tracker_visualizer',
            executable='tracker_visualizer',
            name='visualizer_node',
            parameters=[params["visualizer_node"]['ros__parameters']]
        ),
        # Node(
        #     package='ros_tcp_endpoint',
        #     executable='default_server_endpoint',
        #     name='unity_connector',
        #     parameters=[params['default_server_endpoint']['ros__parameters']]
        # ),
    ])
