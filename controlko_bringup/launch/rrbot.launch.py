from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 引数の宣言
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="controlko_bringup",
            description="configフォルダ内にコントローラーの設定yamlファイルを含むパッケージ名を指定",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="rrbot_controllers.yaml",
            description="コントローラーの設定yamlファイル名",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="controlko_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rrbot.urdf.xacro",
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
            "use_mock_hardware",
            default_value="true",
            description="コマンドをそのまま状態に反映するモックハードウェアを使用するかどうか",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="モックハードウェアでセンサーのcommand interfaceを有効にするかどうか",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            # default_value="forward_position_controller",
            default_value="displacement_controller",
            choices=[
                "forward_position_controller",
                "joint_trajectory_controller",
                "displacement_controller",
            ],
            description="使用するロボットコントローラーの種類",
        )
    )

    # 引数の取得
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

    # xacro経由でURDFを読み込む
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            # ここで引数を渡す
            "prefix:=",
            prefix,
            " use_mock_hardware:=",
            use_mock_hardware,
            " mock_sensor_commands:=",
            mock_sensor_commands,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    # コントローラーの設定ファイルを読み込む
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controllers_file,
        ]    )

    # rviz
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "rrbot.rviz"])

    # controller_managerの起動
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            robot_controllers,
        ],
    )

    # joint_state_broadcasterの起動
    # joint_state_broadcaster はノードじゃなくて、controller_manager が管理するコントローラーのひとつ
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # controllerの起動
    robot_controllers = [robot_controller]  # 複数起動に対応（ここでは単体）
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "-c",
                    "/controller_manager",
                ],
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

    # 遅延実行設定 ---

    # ros2_control_nodeが起動してからjoint_state_broadcasterを起動する
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[joint_state_broadcaster_spawner],  # 1秒後にjoint_state_broadcasterを起動
                )
            ],
        )
    )

    # joint_state_broadcasterが起動してからrvizを起動する
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(  # spawnerが終了したら
            target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]
        )
    )

    # joint_state_broadcasterが起動してからcontrollerを起動する
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[TimerAction(period=3.0, actions=[controller])],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
