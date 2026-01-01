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
    hri_params = PathJoinSubstitution([bringup_share, "config", "hri.yaml"])
    profile_params = PathJoinSubstitution(
        [
            bringup_share,
            "config",
            "profiles",
            PythonExpression([profile, " + '.yaml'"])
        ]
    )

    voice_router = Node(
        package="axon_hri_voice",
        executable="voice_router_node",
        name="voice_router",
        namespace=namespace,
        parameters=[hri_params, profile_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    tts_stub = Node(
        package="axon_hri_voice",
        executable="tts_stub_node",
        name="tts_stub",
        namespace=namespace,
        parameters=[hri_params, profile_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    stt_stub = Node(
        package="axon_hri_voice",
        executable="stt_stub_node",
        name="stt_stub",
        namespace=namespace,
        parameters=[hri_params, profile_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("profile", default_value="dev"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction([PushRosNamespace(namespace), voice_router, tts_stub, stt_stub]),
        ]
    )
