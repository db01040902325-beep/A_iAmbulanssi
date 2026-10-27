"""침대의 heave-pitch 기구학. 소각도, 전후 지지점이 같은 좌우 높이라고 가정."""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Geometry:
    support_spacing_m: float = 0.20  # 앞/뒤 지지점 사이 거리
    stroke_limit_m: float = 0.025    # 중립 위치 기준, 각 구동부 +/-25 mm

    def __post_init__(self):
        if not (isfinite(self.support_spacing_m) and self.support_spacing_m > 0):
            raise ValueError('support_spacing_m must be positive')
        if not (isfinite(self.stroke_limit_m) and self.stroke_limit_m > 0):
            raise ValueError('stroke_limit_m must be positive')

    def to_actuators(self, heave_offset_m: float, pitch_offset_rad: float) -> tuple[float, float]:
        """앞/뒤 구동부의 중립 기준 상대 변위(m)를 반환."""
        half = self.support_spacing_m / 2.0
        return heave_offset_m + half * pitch_offset_rad, heave_offset_m - half * pitch_offset_rad

    def from_actuators(self, front_m: float, rear_m: float) -> tuple[float, float]:
        return (front_m + rear_m) / 2.0, (front_m - rear_m) / self.support_spacing_m


def saturate(x: float, limit: float) -> float:
    return min(max(x, -limit), limit)
