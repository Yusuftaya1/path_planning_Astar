from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('path_planning_robot')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz', 'map_view.rviz')
    
    return LaunchDescription([
        # Static TF publisher for map frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),
        # Map publisher node
        Node(
            package='path_planning_robot',
            executable='occupancy_map_publisher',
            name='occupancy_map_publisher',
            output='screen'
        ),
        # A* planner node
        Node(
            package='path_planning_robot',
            executable='a_star_planner',
            name='a_star_planner',
            output='screen'
        ),
        # RViz2 with custom configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        )
    ]) 