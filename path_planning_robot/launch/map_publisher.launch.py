from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'default']
        )
    ]) 