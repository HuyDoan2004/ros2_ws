from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

# ADD: dùng xacro + robot_state_publisher (giữ nguyên các phần khác)
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')

    cam_cfg   = PathJoinSubstitution([pkg_share, 'configs', 'realsense.yaml'])
    rtab_cfg  = PathJoinSubstitution([pkg_share, 'configs', 'rtabmap.yaml'])
    rviz_cfg  = PathJoinSubstitution([pkg_share, 'configs', 'mapping.rviz'])
    # >>> ADD: rplidar.yaml path
    rplidar_cfg = PathJoinSubstitution([pkg_share, 'configs', 'rplidar.yaml'])

    # ADD: URDF/Xacro (xuất TF từ mô hình)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'walle.urdf.xacro'])
    xacro_exe = FindExecutable(name='xacro')
    body_mesh_uri = 'package://my_robot/meshes/walle_body.dae'

    robot_description = Command([
        xacro_exe, ' ',
        urdf_file, ' ',
        'total_height:=', '0.80', ' ',
        'body_mesh:=',    body_mesh_uri, ' ',
        'body_scale:=',   '1.0'
    ])

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Realsense camera
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
            'unite_imu_method': '2',             # ✅ thêm
            'align_depth.enable': 'true',        # ✅ thêm
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
        }.items()
    )

    # LiDAR node
    lidar = Node(
        package='my_robot', executable='rplidar_node',
        name='rplidar_node', output='screen', emulate_tty=True,
        parameters=[ParameterFile(rplidar_cfg, allow_substs=True),
                    {'frame_id': 'lidar_link'}]
    )

    # YOLO (giữ nguyên)
    cam = Node(
        package='my_robot', executable='realsense_yolo_node',
        name='realsense_yolo_node', output='screen', emulate_tty=True, respawn=True,
        parameters=[ParameterFile(cam_cfg, allow_substs=True)],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_raw', '/camera/depth/image_rect_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
            ('/camera/imu', '/camera/imu'),
        ]
    )

    # Odometry chính dựa trên LiDAR (scan matching ICP)
    icp_odom = Node(
        package='rtabmap_odom', executable='icp_odometry',
        name='icp_odometry', output='screen', emulate_tty=True,
        parameters=[
            ParameterFile(rtab_cfg, allow_substs=True),
            {
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_scan': True,
            }
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # SLAM (map -> odom, fuse LiDAR + RGB-D)
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', output='screen', emulate_tty=True,
        parameters=[
            ParameterFile(rtab_cfg, allow_substs=True),
            {
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'subscribe_odom_info': True,
                'publish_occupancy_grid': True
            }
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan', '/scan'),
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([  
        joint_state_pub,
        robot_state_pub,
        realsense_launch,
        lidar,
        cam,
        TimerAction(period=2.0, actions=[icp_odom]),
        TimerAction(period=2.0, actions=[rtabmap]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
