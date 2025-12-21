from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = [
        "config/pi_core.yaml",
    ]
    return LaunchDescription(
        [
            Node(package="axon_power", executable="power_node", parameters=params),
            Node(package="axon_audio", executable="audio_node", parameters=params),
            Node(package="axon_pwm", executable="pwm_node", parameters=params),
            Node(package="axon_system", executable="diagnostics_node", parameters=params),
            Node(package="axon_system", executable="mode_manager_node", parameters=params),
            Node(package="axon_system", executable="watchdog_node", parameters=params),
        ]
    )
