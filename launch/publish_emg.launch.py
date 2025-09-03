from launch_ros.actions import Node  # type: ignore
from launch_ros.substitutions import FindPackageShare  # type: ignore

from launch import LaunchDescription  # type: ignore
from launch.substitutions import PathJoinSubstitution  # type: ignore


def generate_launch_description() -> LaunchDescription:
    # locate the YAML config in this package
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('shimmer_ros2'),
            'config',
            'shimmer_config.yaml',
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='shimmer_ros2',
                executable='shimmer_publisher',
                name='shimmer_publisher',
                output='screen',
                parameters=[config_file],
            )
        ]
    )
