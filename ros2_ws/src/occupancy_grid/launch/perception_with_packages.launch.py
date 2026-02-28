from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='occupancy_grid',
            executable='pointcloud_to_occupancy_grid_node',
            name='pointcloud_to_grid',
            output='screen',
            parameters=[{
                'pointcloud_topic': '/camera/pointcloud',
                'output_topic': '/occupancy_grid',
                'global_frame': 'world',
                'resolution': 0.05,
                'grid_width': 20.0,
                'grid_height': 20.0,
                'min_obstacle_height': 0.1,
                'max_obstacle_height': 2.5,
                'ground_height_tolerance': 0.1
            }]
        )
    ])