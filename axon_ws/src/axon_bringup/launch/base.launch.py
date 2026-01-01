from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    profile = LaunchConfiguration("profile")
    log_level = LaunchConfiguration("log_level")

    bringup_share = FindPackageShare("axon_bringup")
    base_params = PathJoinSubstitution([bringup_share, "config", "base.yaml"])
    profile_params = PathJoinSubstitution(
        [
            bringup_share,
            "config",
            "profiles",
            PythonExpression([profile, " + '.yaml'"]),
        ]
    )

    node = Node(
        package="axon_base_driver",
        executable="base_node",
        name="base_driver",
        namespace=namespace,
        parameters=[base_params, profile_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("profile", default_value="dev"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction([PushRosNamespace(namespace), node]),
        ]
    )
