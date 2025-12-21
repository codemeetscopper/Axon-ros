from dataclasses import dataclass


@dataclass
class PowerSample:
    bus_v: float
    bus_a: float
    battery1_v: float
    battery1_pct: float
    battery2_v: float
    battery2_pct: float
    ups1_present: bool
    ups2_present: bool
    external_power_present: bool
    on_battery: bool


class PowerBackend:
    def read(self) -> PowerSample:
        raise NotImplementedError
