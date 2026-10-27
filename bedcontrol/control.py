"""실험용 예측 + 피드백 제어기. 실제 환자/차량에 사용하는 의료 제어기가 아님."""
from dataclasses import dataclass
from math import isfinite
from .geometry import Geometry, saturate


@dataclass(frozen=True)
class BedState:
    """세계 좌표에서 침대 중심의 높이/기울기 및 속도 (실험에서는 모의 센서값)."""
    z_m: float = 0.0
    pitch_rad: float = 0.0
    vz_m_s: float = 0.0
    pitch_rate_rad_s: float = 0.0


@dataclass(frozen=True)
class RoadForecast:
    """구급차 바닥(기준면)의 미래 상하 변위와 피치 (실험용 예측값)."""
    base_z_m: float = 0.0
    base_pitch_rad: float = 0.0


@dataclass(frozen=True)
class Setpoint:
    front_m: float
    rear_m: float


@dataclass(frozen=True)
class ControlSettings:
    kp_z: float = 0.75
    kd_z_s: float = 0.045
    kp_pitch: float = 0.8
    kd_pitch_s: float = 0.04
    max_rate_m_s: float = 0.10
    max_measured_pitch_rad: float = 0.35
    max_measured_heave_m: float = 0.08


class BedController:
    def __init__(self, geometry: Geometry, settings: ControlSettings | None = None):
        self.geom = geometry
        self.cfg = settings or ControlSettings()
        self._last_setpoint = Setpoint(0.0, 0.0)

    def reset(self) -> None:
        self._last_setpoint = Setpoint(0.0, 0.0)

    def update(self, state: BedState, forecast: RoadForecast, dt_s: float,
               *, enable: bool = True) -> Setpoint:
        values = (state.z_m, state.pitch_rad, state.vz_m_s, state.pitch_rate_rad_s,
                  forecast.base_z_m, forecast.base_pitch_rad, dt_s)
        if not all(isfinite(x) for x in values) or dt_s <= 0 or dt_s > 0.2:
            raise ValueError('invalid sensor/forecast/time data')
        if abs(state.pitch_rad) > self.cfg.max_measured_pitch_rad or abs(state.z_m) > self.cfg.max_measured_heave_m:
            raise ValueError('sensor out of permitted simulation envelope')
        # AI 모델이 아직 없으므로 forecast는 시뮬레이션 데이터만 사용합니다.
        # 몸체 운동을 0으로 만드는 상대 높이 명령(높이 제어 + 피치 제어).
        if enable:
            z_cmd = (-forecast.base_z_m - self.cfg.kp_z * state.z_m
                     - self.cfg.kd_z_s * state.vz_m_s)
            th_cmd = (-forecast.base_pitch_rad - self.cfg.kp_pitch * state.pitch_rad
                      - self.cfg.kd_pitch_s * state.pitch_rate_rad_s)
            f_goal, r_goal = self.geom.to_actuators(z_cmd, th_cmd)
        else:
            f_goal, r_goal = 0.0, 0.0
        limit = self.geom.stroke_limit_m
        f_goal, r_goal = saturate(f_goal, limit), saturate(r_goal, limit)
        # 기구 손상 방지를 위한 변위 및 매 반복 명령 속도 제한.
        step = self.cfg.max_rate_m_s * dt_s
        f = self._last_setpoint.front_m + saturate(f_goal - self._last_setpoint.front_m, step)
        r = self._last_setpoint.rear_m + saturate(r_goal - self._last_setpoint.rear_m, step)
        self._last_setpoint = Setpoint(saturate(f, limit), saturate(r, limit))
        return self._last_setpoint
