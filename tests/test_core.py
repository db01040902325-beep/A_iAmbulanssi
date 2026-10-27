import math
import unittest
from bedcontrol.geometry import Geometry
from bedcontrol.control import BedController, BedState, RoadForecast
from bedcontrol.simulation import simulate, summary


class GeometryTest(unittest.TestCase):
    def test_heave_moves_both_ends_equally(self):
        front, rear = Geometry().to_actuators(0.01, 0.0)
        self.assertAlmostEqual(front, 0.01)
        self.assertAlmostEqual(rear, 0.01)

    def test_pitch_moves_ends_oppositely(self):
        f, r = Geometry().to_actuators(0, 0.1)
        self.assertAlmostEqual(f, 0.01)
        self.assertAlmostEqual(r, -0.01)
        self.assertAlmostEqual(Geometry().from_actuators(f, r)[1], 0.1)

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):
            Geometry(support_spacing_m=0)


class ControllerTest(unittest.TestCase):
    def test_rate_limited(self):
        c = BedController(Geometry())
        p = c.update(BedState(), RoadForecast(0.1, 0), 0.01)
        self.assertAlmostEqual(p.front_m, -0.001)
        self.assertAlmostEqual(p.rear_m, -0.001)

    def test_stroke_limited(self):
        c = BedController(Geometry())
        for _ in range(100):
            result = c.update(BedState(), RoadForecast(0.1, 0.1), 0.01)
        self.assertLessEqual(abs(result.front_m), 0.025)
        self.assertLessEqual(abs(result.rear_m), 0.025)

    def test_nonfinite_sensor_rejected(self):
        with self.assertRaises(ValueError):
            BedController(Geometry()).update(BedState(z_m=math.nan), RoadForecast(), 0.01)

    def test_disable_neutral(self):
        c = BedController(Geometry())
        c.update(BedState(), RoadForecast(0.01, 0), 0.01)
        for _ in range(40):
            p = c.update(BedState(), RoadForecast(0.01, 0), 0.01, enable=False)
        self.assertAlmostEqual(p.front_m, 0, places=6)
        self.assertAlmostEqual(p.rear_m, 0, places=6)


class SimulationTest(unittest.TestCase):
    def test_preview_improves_on_passive_for_demo_signal(self):
        passive = summary(simulate('passive'))
        preview = summary(simulate('preview'))
        self.assertLess(preview['heave_rms_mm'], passive['heave_rms_mm'])
        self.assertLess(preview['pitch_rms_deg'], passive['pitch_rms_deg'])

    def test_reject_unknown_mode(self):
        with self.assertRaises(ValueError):
            simulate('unknown')


if __name__ == '__main__':
    unittest.main()

# test runner with unittest discover also finds appended tests.
from bedcontrol.dynamics import LoadParameters, required_vertical_forces


class DynamicsTest(unittest.TestCase):
    def test_static_supports_share_weight(self):
        load = LoadParameters(1.0, 0.01)
        a, b = required_vertical_forces(Geometry(), load, 0, 0)
        self.assertAlmostEqual(a, 9.80665 / 2)
        self.assertAlmostEqual(b, 9.80665 / 2)

    def test_pitch_acceleration_creates_force_difference(self):
        geometry = Geometry(support_spacing_m=0.4)
        load = LoadParameters(2.0, 0.02)
        f, r = required_vertical_forces(geometry, load, 1.0, 3.0)
        self.assertAlmostEqual(f + r, 2.0 * (9.80665 + 1.0))
        self.assertAlmostEqual((geometry.support_spacing_m / 2) * (f - r), 0.02 * 3.0)
