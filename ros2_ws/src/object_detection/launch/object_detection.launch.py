from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='ObjectMappingToWorld',
            name='object_to_world',
            output='screen',
            remappings= [
                ('semantic_camera_info','/Quadrotor/Sensors/SemanticCamera/camera_info'),
                ('semantic_image', '/Quadrotor/Sensors/SemanticCamera/image_raw'),
                ('depth_image', '/realsense/depth/image')
                ]
        )
    ])