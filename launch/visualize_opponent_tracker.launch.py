import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('opponent_tracker')
    config_path = os.path.join(pkg_dir, 'config', 'opponent_tracker_params.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'opponent_tracker.rviz')

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/dynamic_cloud',
        description='Dynamic point cloud topic'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (rosbag) clock if true'
    )

    tracker_node = Node(
        package='opponent_tracker',
        executable='opponent_tracker_node',
        name='opponent_tracker_node',
        output='screen',
        parameters=[
            config_path,
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        use_sim_time_arg,
        tracker_node,
        rviz_node
    ])

