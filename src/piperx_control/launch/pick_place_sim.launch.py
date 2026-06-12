from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    settle_velocity_threshold = LaunchConfiguration('settle_velocity_threshold')
    marker_size = LaunchConfiguration('marker_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'settle_velocity_threshold',
            default_value='0.033',
            description='Joint velocity threshold used to decide when the arm is settled'
        ),

        DeclareLaunchArgument(
            'marker_size',
            default_value='0.055',
            description='ArUco marker side length in meters'
        ),

        Node(
            package='piperx_perception',
            executable='aruco_sim_detector',
            name='aruco_sim_detector',
            output='screen',
            parameters=[
                {'marker_size': marker_size}
            ]
        ),

        Node(
            package='piperx_control',
            executable='piperx_sim_control',
            name='piperx_sim_control',
            output='screen',
            parameters=[
                {'settle_velocity_threshold': settle_velocity_threshold}
            ]
        )
    ])
