"""수직 2자유도 강체 방정식 (간이 모델, 모터 선정의 초기 기준).

       F_front + F_rear - mg = m * a_z
       (L/2)*(F_front - F_rear) = I_pitch * alpha_pitch

F는 수직방향 작용력. 실제 조인트 각도/마찰/탄성·다른 하중 경로는 미포함.
"""
from dataclasses import dataclass
from math import isfinite
from .geometry import Geometry


@dataclass(frozen=True)
class LoadParameters:
    total_mass_kg: float = 0.8
    pitch_inertia_kg_m2: float = 0.003
    gravity_m_s2: float = 9.80665

    def __post_init__(self):
        if not all(isfinite(x) and x > 0 for x in (
            self.total_mass_kg, self.pitch_inertia_kg_m2, self.gravity_m_s2
        )):
            raise ValueError('load parameters must be positive and finite')


def required_vertical_forces(geometry: Geometry, load: LoadParameters,
                             z_acceleration_m_s2: float, pitch_alpha_rad_s2: float) -> tuple[float, float]:
    """전후 구동부가 담당해야 할 수직방향 합력 분담(N). 음수는 인장력이 필요하다는 의미."""
    if not (isfinite(z_acceleration_m_s2) and isfinite(pitch_alpha_rad_s2)):
        raise ValueError('accelerations must be finite')
    fsum = load.total_mass_kg * (z_acceleration_m_s2 + load.gravity_m_s2)
    moment = load.pitch_inertia_kg_m2 * pitch_alpha_rad_s2
    f_front = 0.5 * fsum + moment / geometry.support_spacing_m
    f_rear = 0.5 * fsum - moment / geometry.support_spacing_m
    return f_front, f_rear
