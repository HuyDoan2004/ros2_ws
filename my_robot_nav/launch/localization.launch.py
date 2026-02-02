from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Localization stack using RTAB-Map + my_robot sensors.

    - Loads an existing RTAB-Map database (.db) for localization-only mode.
    - Starts RealSense + LiDAR drivers and the required static TFs.
    """

    pkg_share = get_package_share_directory('my_robot')

    rtab_cfg = PathJoinSubstitution([pkg_share, 'configs', 'rtabmap.yaml'])
    rplidar_cfg = PathJoinSubstitution([pkg_share, 'configs', 'rplidar.yaml'])

    # Path to RTAB-Map database generated during mapping
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='~/ros2_ws/maps/my_home.db',
        description='Path to RTAB-Map database (.db) used for localization',
    )
    map_file = LaunchConfiguration('map')

    # RealSense Camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'wait_for_device_timeout': '15.0',
            'initial_reset': 'true',
            'enable_sync': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
        }.items(),
    )

    # LiDAR
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

    # RTAB-Map in localization-only mode, loading the given database
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        emulate_tty=True,
        parameters=[
            ParameterFile(rtab_cfg, allow_substs=True),
            {
                'Mem/IncrementalMemory': 'false',  # localization-only
                'Mem/InitWMWithAllNodes': 'true',  # load full map in memory
                'database_path': map_file,
                'delete_db_on_start': False,
            },
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
        ],
    )

    # Static TFs (if you don't use robot_state_publisher here)
    tf_base2cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.30', '0', '0', '0', 'base_link', 'camera_link'],
    )

    tf_base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'lidar_link'],
    )

    return LaunchDescription([
        map_arg,
        realsense_launch,
        lidar,
        rtabmap,
        tf_base2cam,
        tf_base2lidar,
    ])
