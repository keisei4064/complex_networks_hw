from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 引数の宣言
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="controlko_description",
            description="これにより、デフォルトのものではなくカスタムdescriptionパッケージを使用できます。",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="複数ロボットセットアップに役立つジョイント名のプレフィックス",
        )
    )

    # 引数の取得
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")

    # xacro経由でURDFを読み込む
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "rrbot.urdf.xacro"]),
            " ",
            "prefix:=",  # ここでプレフィックスを渡す
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "rviz",
            "rrbot.rviz",
        ]
    )

    # 必要なノードの定義
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
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

    # rvizは遅延実行する
    delay_rviz_after_joint_state_publisher_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_publisher_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[rviz_node],  # rvizを2秒後に起動
                )
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_publisher_node,
        ]
    )
