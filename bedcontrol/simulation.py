"""도로 표면 입력에 따른 단순 2자유도 기구 모의실험.

주의: 수직 상대변위를 액추에이터로 즉시/1차 지연 추종하는 단순 모델입니다.
실제 힘/질량/스프링/댐퍼/모터 동역학은 아직 모델링하지 않았습니다.
"""
from dataclasses import dataclass
from math import exp, pi, sin
from .geometry import Geometry, saturate
from .control import BedController, BedState, RoadForecast


@dataclass(frozen=True)
class SimulationSettings:
    dt_s: float = 0.01
    duration_s: float = 12.0
    # 이 값은 모터 시험 전 '가정한' 액추에이터 응답 지연입니다.
    actuator_time_constant_s: float = 0.045


def road_motion(t: float) -> RoadForecast:
    """재현 가능한 가상의 도로 요철. AI 학습 결과가 아님."""
    bumps = (0.009 * exp(-((t - 3.0) / 0.23) ** 2)
             - 0.010 * exp(-((t - 6.8) / 0.19) ** 2)
             + 0.007 * exp(-((t - 9.5) / 0.3) ** 2))
    z = 0.0045 * sin(2 * pi * 0.75 * t) + bumps
    th = 0.019 * sin(2 * pi * 0.42 * t) + 0.025 * exp(-((t - 6.75) / 0.35) ** 2)
    return RoadForecast(z, th)


def simulate(mode: str = 'preview', settings: SimulationSettings | None = None) -> list[dict[str, float]]:
    """mode: passive (제어 없음), feedback (미래 미예측), preview (가상 미래 노면 정보).

    preview는 'AI 성능'을 의미하지 않습니다. 테스트 상한값에 가까운 이상적 미래 입력입니다.
    """
    if mode not in ('passive', 'feedback', 'preview'):
        raise ValueError('mode must be passive, feedback or preview')
    cfg = settings or SimulationSettings()
    if cfg.dt_s <= 0 or cfg.duration_s <= 0 or cfg.actuator_time_constant_s <= 0:
        raise ValueError('simulation settings must be positive')
    geom = Geometry()
    controller = BedController(geom)
    front = rear = 0.0
    prev_z = prev_pitch = 0.0
    records = []
    # simulated actuator lag: first order, not a calibrated real motor model
    alpha = cfg.dt_s / (cfg.actuator_time_constant_s + cfg.dt_s)
    n = round(cfg.duration_s / cfg.dt_s)
    for i in range(n):
        t = i * cfg.dt_s
        base = road_motion(t)
        offset_z, offset_pitch = geom.from_actuators(front, rear)
        bed_z = base.base_z_m + offset_z
        bed_pitch = base.base_pitch_rad + offset_pitch
        # 초기 첫 표본에는 0으로 놓음
        state = BedState(bed_z, bed_pitch,
                         (bed_z - prev_z) / cfg.dt_s if i else 0.0,
                         (bed_pitch - prev_pitch) / cfg.dt_s if i else 0.0)
        if mode == 'preview':
            future = road_motion(t + cfg.actuator_time_constant_s)
        elif mode == 'feedback':
            future = RoadForecast()
        else:
            future = RoadForecast()
        target = controller.update(state, future, cfg.dt_s, enable=(mode != 'passive'))
        front = saturate(front + alpha * (target.front_m - front), geom.stroke_limit_m)
        rear = saturate(rear + alpha * (target.rear_m - rear), geom.stroke_limit_m)
        records.append({
            'time_s': t, 'base_z_mm': base.base_z_m * 1000,
            'bed_z_mm': bed_z * 1000, 'base_pitch_deg': base.base_pitch_rad * 180 / pi,
            'bed_pitch_deg': bed_pitch * 180 / pi,
            'front_target_mm': target.front_m * 1000, 'rear_target_mm': target.rear_m * 1000,
            'front_actual_mm': front * 1000, 'rear_actual_mm': rear * 1000,
        })
        prev_z, prev_pitch = bed_z, bed_pitch
    return records


def rms(values) -> float:
    data = list(values)
    return (sum(x * x for x in data) / len(data)) ** 0.5 if data else 0.0


def summary(rows: list[dict[str, float]]) -> dict[str, float]:
    return {
        'heave_rms_mm': rms(row['bed_z_mm'] for row in rows),
        'pitch_rms_deg': rms(row['bed_pitch_deg'] for row in rows),
    }
