from launch import LaunchDescription
from launch_ros.actions import Node




def generate_launch_description():
    # Camera to Quadrotor/TrueState (adjust based on your camera position on the drone)
    camera_to_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',     
            '--y', '0.0',
            '--z', '0.1',   #distance Quadrotor/TrueState center to camera
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'Quadrotor/TrueState',
            '--child-frame-id', 'Quadrotor/Sensors/DepthCamera_optical'
        ]
    )
    
    # Align depth camera with semantic camera (identity transform)
    depth_to_semantic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'Quadrotor/Sensors/SemanticCamera',
            '--child-frame-id', 'Quadrotor/Sensors/DepthCamera_optical'
        ]
    )

    object_mapping_node = Node(
        package='object_detection',
        executable='ObjectMappingToWorld',
        name='object_to_world',
        output='screen',
        remappings=[
            ('semantic_camera_info', '/Quadrotor/Sensors/SemanticCamera/camera_info'),
            ('semantic_image', '/Quadrotor/Sensors/SemanticCamera/image_raw'),
            ('depth_image', '/realsense/depth/image')
        ]
    )
    
    return LaunchDescription([
        camera_to_body,
        depth_to_semantic,
        object_mapping_node
    ])




