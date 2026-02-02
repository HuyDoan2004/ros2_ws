from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """RTAB-Map SLAM 3D dùng cả LiDAR + depth camera (RealSense)."""
    my_robot_share = get_package_share_directory('my_robot')

    # Cấu hình LiDAR (hardware giữ trong yaml cho tiện chỉnh cổng/baud)
    rplidar_cfg = PathJoinSubstitution([my_robot_share, 'configs', 'rplidar.yaml'])

    # --- Sensor nodes ---

    # LiDAR driver from my_robot package
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

    # Depth camera: RealSense D435i
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

    # Static TFs giữa base_link, camera_link, lidar_link
    tf_base2cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        arguments=['0.0', '0.0', '0.30', '0', '0', '0', 'base_link', 'camera_link'],
    )

    tf_base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_lidar',
        arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'lidar_link'],
    )

    # --- RTAB-Map RGBD Odometry (3D) ---
    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                # Frame gốc của cảm biến RGBD
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_rgbd': False,
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_imu': True,
                'approx_sync': True,
                'imu_topic': '/camera/imu',
                # Tham số ví dụ, có thể chỉnh trực tiếp trong code:
                'Reg/Strategy': '1',            # 0: Visual, 1: ICP+visual, 2: ICP-only
                'Odom/GuessMotion': 'true',
                'Odom/ResetCountdown': '0',
            },
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
        ],
    )

    # --- RTAB-Map SLAM 3D (fuse RGBD + LiDAR) ---
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_3d_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'subscribe_odom_info': True,
                'publish_occupancy_grid': True,
                # Bật các subscription 3D (RGB + depth + LiDAR scan)
                'subscribe_scan': True,
                'subscribe_rgbd': False,
                'subscribe_rgb': True,
                'subscribe_depth': True,
                # Cấu hình SLAM 3D: grid 3D, voxel size, v.v.
                'Grid/3D': 'true',
                'Grid/RangeMax': '8.0',
                'Grid/CellSize': '0.05',
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'Rtabmap/DetectionRate': '3.0',
            },
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        # Sensors
        lidar,
        realsense_launch,
        tf_base2cam,
        tf_base2lidar,
        # SLAM pipeline (trễ nhẹ để sensor lên trước)
        TimerAction(period=2.0, actions=[rgbd_odom]),
        TimerAction(period=3.0, actions=[rtabmap]),
    ])
