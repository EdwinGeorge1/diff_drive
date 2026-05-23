import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    # =====================================
    # Package Paths
    # =====================================

    pkg_path = get_package_share_directory(
        'diffdrive_description'
    )

    robot_localization_path = get_package_share_directory(
        'robot_localization'
    )

    # =====================================
    # URDF / XACRO
    # =====================================

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'my_robot.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        xacro_file
    )

    robot_description = {
        'robot_description':
        robot_description_config.toxml()
    }

    # =====================================
    # RViz Config
    # =====================================

    rviz_config = os.path.join(
        pkg_path,
        'rviz',
        'diffdrive_config.rviz'
    )

    # =====================================
    # Robot State Publisher
    # =====================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # =====================================
    # Joint State Publisher
    # =====================================

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # =====================================
    # EKF Launch
    # =====================================

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_localization_path,
                'launch',
                'ekf_new.launch.py'
            )
        )
    )

    # =====================================
    # RViz Node
    # =====================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # =====================================
    # Launch Description
    # =====================================

    return LaunchDescription([

        robot_state_publisher_node,

        joint_state_publisher_node,

        ekf_launch,

        rviz_node

    ])