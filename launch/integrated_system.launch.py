import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tracker_dir = get_package_share_directory('opponent_tracker')
    pkg_gn10_dir = get_package_share_directory('gn10_pointcloud_localization')

    tracker_config = os.path.join(pkg_tracker_dir, 'config', 'opponent_tracker_params.yaml')
    rviz_config = os.path.join(pkg_tracker_dir, 'rviz', 'opponent_tracker.rviz')
    gn10_config = os.path.join(pkg_gn10_dir, 'config', 'localization_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (rosbag) clock if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    # 1. Static TF (base_link -> livox_frame)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=[
            '--x', '-0.2',
            '--y', '0.25',
            '--z', '1.09',
            '--yaw', '3.1415',
            '--pitch', '-0.273',
            '--roll', '3.13',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 2. gn10 Localization Node
    gn10_node = Node(
        package='gn10_pointcloud_localization',
        executable='gn10_pointcloud_localization_node',
        name='gn10_pointcloud_localization_node',
        parameters=[
            gn10_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # 3. Opponent Tracker Node
    tracker_node = Node(
        package='opponent_tracker',
        executable='opponent_tracker_node',
        name='opponent_tracker_node',
        parameters=[
            tracker_config,
            {
                'input_topic': '/dynamic_cloud',
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        static_tf_node,
        gn10_node,
        tracker_node,
        rviz_node
    ])

