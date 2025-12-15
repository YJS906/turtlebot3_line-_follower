from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera viewer node from rover_line_follower package
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'jump_reject_threshold': 360.0,
        }]
    )

    # Line follower node from rover_line_follower package
    line_follower_node = Node(
        package='rover_line_follower',
        executable='line_follower',
        name='line_follower',
        output='screen',
        parameters=[{
            'wait_seconds': 20.0,
            'linear_speed': 2.62,
            'angular_scale': 0.004,
            'angular_turn_rate': 1.6,
            'coverage_threshold': 0.230,
            'max_acceleration': 2.62,
            'max_error': 900.0,
            'min_linear_speed': 1.5,
            'no_line_left_speed': -0.17,
            'no_line_right_speed': 0.17,
            'wheel_separation': 0.287,
            'no_line_linear_speed': 0.08,
        }]
    )

    # RQT image view
    rqt_image = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/line_follower/debug_image']
    )

    return LaunchDescription([
        camera_viewer_node,
        line_follower_node,
        rqt_image
    ])
