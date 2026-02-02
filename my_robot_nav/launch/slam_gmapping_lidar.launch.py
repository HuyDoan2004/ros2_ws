from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """2D LiDAR SLAM using GMapping (slam_gmapping) on /scan."""
    my_robot_share = get_package_share_directory('my_robot')

    rplidar_cfg = PathJoinSubstitution([my_robot_share, 'configs', 'rplidar.yaml'])

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

    gmapping = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[
            {
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
            },
        ],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        lidar,
        tf_base2lidar,
        TimerAction(period=2.0, actions=[gmapping]),
    ])
