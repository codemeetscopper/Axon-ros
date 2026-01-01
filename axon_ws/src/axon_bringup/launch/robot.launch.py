from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    profile = LaunchConfiguration("profile")
    log_level = LaunchConfiguration("log_level")

    bringup_share = FindPackageShare("axon_bringup")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "base.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "profile": profile,
            "log_level": log_level,
        }.items(),
    )

    audio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "audio.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "profile": profile,
            "log_level": log_level,
        }.items(),
    )

    hri_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "hri.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "profile": profile,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("profile", default_value="dev"),
            DeclareLaunchArgument("log_level", default_value="info"),
            base_launch,
            audio_launch,
            hri_launch,
        ]
    )
