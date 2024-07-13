import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('fast_lio_sam'), 'config', 'ros2_param.yaml')
    rviz_config = os.path.join(get_package_share_directory('fast_lio_sam'), 'rviz_cfg', 'map_ros2.rviz')

    fast_lio_sam = Node(package="fast_lio_sam", executable="fastlio_sam_ros2", prefix=['stdbuf -o L'], output='screen', 
                        parameters=[config, 
                                    {"showOptimizedPose": True} , 
                                    {"globalMapVisualizationSearchRadius": 1000.}, 
                                    {"globalMapVisualizationPoseDensity": 3.}, 
                                    {"globalMapVisualizationLeafSize": 0.2}])
    # fast_lio_sam = Node(package="fast_lio_sam", executable="fastlio_sam_ros2", prefix=['gdb -ex run --args'], output='screen', parameters=[config])
    rviz2 = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
    return LaunchDescription([fast_lio_sam, rviz2])
