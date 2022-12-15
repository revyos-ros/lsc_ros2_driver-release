import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
  addr = LaunchConfiguration('addr', default='192.168.0.1')
  port = LaunchConfiguration('port', default='8000')
  frame_id = LaunchConfiguration('frame_id', default="laser")
  range_min = LaunchConfiguration('range_min', default=0.05)
  range_max = LaunchConfiguration('range_max', default=25.0)
  intensities = LaunchConfiguration('intensities', default='true')

  pub_topic = LaunchConfiguration('pub_topic', default="scan")
  diagnostics_tolerance = LaunchConfiguration('diagnostics_tolerance', default=0.1)
  diagnostics_windows_time = LaunchConfiguration('diagnostics_windows_time', default=1)


  rviz_config_dir = os.path.join(
    get_package_share_directory('lsc_ros2_driver'),
    'rviz',
    'lsc_ros2_driver.rviz'
  )

  return LaunchDescription([

    DeclareLaunchArgument(
      'addr',
      default_value=addr,
      description='server address to connect'
    ),

    DeclareLaunchArgument(
      'port',
      default_value=port,
      description='server port opened'
    ),

    DeclareLaunchArgument(
      'frame_id',
      default_value=frame_id,
      description='topic frame_id'
    ),

    DeclareLaunchArgument(
      'range_min',
      default_value=range_min,
      description='minimum range value of device'
    ),

    DeclareLaunchArgument(
      'range_max',
      default_value=range_max,
      description='maximum range value of device'
    ),

    DeclareLaunchArgument(
      'intensities',
      default_value=intensities,
      description='intensities enable'
    ),

    DeclareLaunchArgument(
      'pub_topic',
      default_value=pub_topic,
      description='topic name'
    ),

    DeclareLaunchArgument(
      'diagnostics_tolerance',
      default_value=diagnostics_tolerance,
      description='set diagnostics_tolerance value'
    ),

    DeclareLaunchArgument(
      'diagnostics_windows_time',
      default_value=diagnostics_windows_time,
      description='set diagnostics_windows_time value'
    ),

    Node(
      package='lsc_ros2_driver',
      executable='autonics_lsc_lidar',
      name='autonics_lsc_lidar',
      parameters=[{
        'addr': addr,
        'port': port,
        'frame_id' : frame_id,
        'range_min': range_min,
        'range_max': range_max,
        'intensities': intensities,
        'pub_topic' : pub_topic,
        'diagnostics_tolerance' : diagnostics_tolerance,
        'diagnostics_windows_time' : diagnostics_windows_time
      }],
      output='screen'),

    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d' + rviz_config_dir],
      output='screen'),
  ])
