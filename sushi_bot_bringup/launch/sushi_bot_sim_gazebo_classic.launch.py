import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 引数宣言
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="sushi_bot_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="sushi_bot_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="sushi_bot_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="sushi_bot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        )
    )

    # 引数初期化
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    # use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "sushi_bot.rviz"])

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=false",
            " ",
            "mock_sensor_commands:=false",
            " ",
            "sim_gazebo_classic:=true",
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]),
    )

    # Spawn robot in Gazebo
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_sushi_bot",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "sushi_bot",
        ],
        output="screen",
    )

    # コントローラー，ブロードキャスターのスポーン
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    # その他のノード
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # 遅延起動 ---
    
    # gazeboのロボットスポーン後にJoint State Broadcasterを起動する
    delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_spawn_robot,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # joint_state_broadcasterの起動後にrvizを起動する
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # joint_state_broadcasterの起動後にrobot_controller_spawnersを起動する
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            gazebo,
            gazebo_spawn_robot,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
