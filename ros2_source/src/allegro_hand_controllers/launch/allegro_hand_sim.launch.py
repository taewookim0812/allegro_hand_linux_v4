from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Locate packages
    allegro_hand_desc = get_package_share_directory('allegro_hand_description')
    allegro_hand_bringup = get_package_share_directory('allegro_hand_controllers')

    # Launch arguments
    declare_visualize_arg = DeclareLaunchArgument(
        'RVIZ',
        default_value='true',
        description='Flag to enable/disable RViz visualization'
    )

    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )

    declare_num_arg = DeclareLaunchArgument(
        'NUM',
        default_value='0',
        description='Specify AH number for remapping topics (for multi-hand setups)'
    )

    # Launch config substitutions
    hand = LaunchConfiguration('HAND')
    num = LaunchConfiguration('NUM')

    # URDF path generation
    urdf_path = PythonExpression([
        '"', allegro_hand_desc, '/urdf/allegro_hand_description_', hand, '.urdf"'
    ])

    # Optional RViz launch
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(allegro_hand_bringup, "launch", "allegro_rviz.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration('RVIZ'))
    )

    return LaunchDescription([
        declare_visualize_arg,
        declare_hand_arg,
        declare_num_arg,
        include_rviz,

        # Simulated joint publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            name='joint_state_publisher'
        ),

        # Publishes robot state from URDF (Xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),
    ])

