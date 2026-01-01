import json

from axon_base_driver.kinematics import DiffDriveKinematics
from axon_base_driver.protocol import SpeedCommand, encode_speed_ctrl


def test_twist_to_wheel_speeds() -> None:
    kin = DiffDriveKinematics(wheel_base=0.3, max_speed=0.5)
    left, right = kin.twist_to_wheel_speeds(0.1, 0.0)
    assert left == right == 0.1


def test_encode_speed_ctrl() -> None:
    payload = encode_speed_ctrl(SpeedCommand(left=0.1, right=0.1))
    data = json.loads(payload.decode("utf-8"))
    assert data["T"] == 1
    assert data["L"] == 0.1
