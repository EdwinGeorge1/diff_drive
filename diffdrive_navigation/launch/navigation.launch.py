import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    # =========================================
    # Launch Configurations
    # =========================================

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    map_yaml_file = LaunchConfiguration(
        'map'
    )

    # =========================================
    # Declare Arguments
    # =========================================

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory(
                'diffdrive_navigation'
            ),
            'maps',
            'simple_map.yaml'
        ),
        description='Full path to map yaml file'
    )

    # =========================================
    # Package Directories
    # =========================================

    nav2_bringup_dir = get_package_share_directory(
        'nav2_bringup'
    )

    diffdrive_navigation_dir = (
        get_package_share_directory(
            'diffdrive_navigation'
        )
    )

    # =========================================
    # Nav2 Parameters File
    # =========================================

    nav2_params_file = os.path.join(
        diffdrive_navigation_dir,
        'config',
        'nav2_params.yaml'
    )

    # =========================================
    # Nav2 Bringup
    # =========================================

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),

        launch_arguments={

            'slam': 'False',

            'map': map_yaml_file,

            'use_sim_time': use_sim_time,

            'params_file': nav2_params_file

        }.items()
    )

    # =========================================
    # Launch Description
    # =========================================

    ld = LaunchDescription()

    ld.add_action(
        declare_use_sim_time_cmd
    )

    ld.add_action(
        declare_map_yaml_cmd
    )

    ld.add_action(
        nav2_bringup_cmd
    )

    return ld