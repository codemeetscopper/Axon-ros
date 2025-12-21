import time
from .base import PowerBackend, PowerSample


class MockPowerBackend(PowerBackend):
    def __init__(self) -> None:
        self._start = time.time()
        self._low_battery = False

    def set_low_battery(self, enabled: bool) -> None:
        self._low_battery = enabled

    def read(self) -> PowerSample:
        uptime = time.time() - self._start
        pct = 0.8 if not self._low_battery else 0.15
        bus_v = 12.2
        bus_a = 1.2
        external_power = uptime % 120 < 90
        on_battery = not external_power
        return PowerSample(
            bus_v=bus_v,
            bus_a=bus_a,
            battery1_v=12.1,
            battery1_pct=pct,
            battery2_v=12.0,
            battery2_pct=pct,
            ups1_present=True,
            ups2_present=True,
            external_power_present=external_power,
            on_battery=on_battery,
        )
