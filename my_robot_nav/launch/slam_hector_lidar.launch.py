from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """2D LiDAR SLAM using Hector Mapping on /scan."""
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

    hector_mapping = Node(
        package='hector_mapping',
        executable='hector_mapping',
        name='hector_mapping',
        output='screen',
        parameters=[
            {
                'pub_map_odom_transform': True,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
            },
        ],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        lidar,
        tf_base2lidar,
        TimerAction(period=2.0, actions=[hector_mapping]),
    ])
