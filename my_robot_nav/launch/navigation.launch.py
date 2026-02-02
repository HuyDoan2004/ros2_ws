from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_share = get_package_share_directory('my_robot_nav')
    
    nav2_params = PathJoinSubstitution([nav_share, 'config', 'nav2_params.yaml'])
    
    # Launch Localization (RTAB-Map + sensors)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_share, 'launch', 'localization.launch.py'])
        )
    )
    
    # Launch Nav2 Stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 
                'launch', 
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )
    
    return LaunchDescription([
        localization,
        nav2_bringup,
    ])
