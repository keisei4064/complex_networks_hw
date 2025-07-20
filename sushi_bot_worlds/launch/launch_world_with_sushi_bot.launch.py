import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare the 'world' argument with a default path
    default_world_path = os.path.join(
        get_package_share_directory("sushi_bot_worlds"),
        "worlds",
        "sushi_pick_and_place.sdf",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Full path to the world file to load",
    )

    world = LaunchConfiguration("world")

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("sushi_bot_bringup"),
                    "launch",
                    "sushi_bot_sim_gazebo_classic.launch.py",
                )
            ]
        ),
        launch_arguments={"world": world, "use_sim_time": "true"}.items(),
    )
    return LaunchDescription([declare_world, gazebo_launch])
