import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('opponent_tracker')
    default_config_path = os.path.join(pkg_dir, 'config', 'opponent_tracker_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to config file for opponent_tracker'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/dynamic_points',
        description='Dynamic point cloud input topic'
    )

    tracker_node = Node(
        package='opponent_tracker',
        executable='opponent_tracker_node',
        name='opponent_tracker_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'input_topic': LaunchConfiguration('input_topic')}
        ]
    )

    return LaunchDescription([
        config_arg,
        input_topic_arg,
        tracker_node
    ])

