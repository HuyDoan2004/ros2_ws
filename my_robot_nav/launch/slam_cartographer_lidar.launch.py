from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """2D LiDAR SLAM using Google Cartographer on /scan."""
    my_robot_share = get_package_share_directory('my_robot')

    # Reuse existing LiDAR config from my_robot
    rplidar_cfg = PathJoinSubstitution([my_robot_share, 'configs', 'rplidar.yaml'])

    carto_share = FindPackageShare('cartographer_ros')
    carto_cfg_dir = PathJoinSubstitution([carto_share, 'config'])
    carto_cfg_basename = 'backpack_2d.lua'  # standard 2D example config

    lidar = Node(
        package='my_robot',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            ParameterFile(rplidar_cfg, allow_substs=True),
            {'frame_id': 'lidar_link'},
        ],
    )

    tf_base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_lidar',
        arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'lidar_link'],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', carto_cfg_dir,
            '-configuration_basename', carto_cfg_basename,
        ],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'resolution': 0.05},
            {'publish_period_sec': 1.0},
        ],
    )

    return LaunchDescription([
        lidar,
        tf_base2lidar,
        TimerAction(period=2.0, actions=[cartographer_node]),
        TimerAction(period=4.0, actions=[occupancy_grid]),
    ])
