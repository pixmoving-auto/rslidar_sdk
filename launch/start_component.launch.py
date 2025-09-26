from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  config_file = os.path.join(get_package_share_directory('robobus_sensor_kit_launch'), 'config', 'robosense_config.yaml')
  rslidar_component = ComposableNode(
    package='rslidar_sdk',
    plugin='robosense::lidar::RSLidarSDKComponent',
    name='rslidar_node',
    parameters=[{
      'config_path': config_file,
    }]
  )
  loader = LoadComposableNodes(
        target_container='pointcloud_container',
        composable_node_descriptions=[rslidar_component]
    )
  return LaunchDescription([loader])