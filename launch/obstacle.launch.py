# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# import os

def generate_launch_description():

    ld = LaunchDescription()


    move_obstacle_node = Node(
        package='cctv_layer_ros2',
        executable='move_obstacle',
        name='move_obstacle',
        )
    
    pub_detection_node = Node(
        package='cctv_layer_ros2',
        executable='pub_detection',
        name='pub_detection',
        )


    ld.add_action(move_obstacle_node)
    ld.add_action(pub_detection_node)
    return ld
