"""Exercise the actual portable C estimator with known body-frame trajectories.

No board motion is required. Inputs/expected rotations are constructed independently
from rotation geometry; this checks conventions, dynamics and fault behavior.
"""
import ctypes as c
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
G = 9.80665


class Sample(c.Structure):
    _fields_ = [("timestamp", c.c_uint32), ("sequence", c.c_uint32), ("dt", c.c_uint32),
                ("accel", c.c_int16 * 3), ("temp", c.c_int16),
                ("gyro", c.c_int16 * 3), ("flags", c.c_uint16)]


class Measurement(c.Structure):
    _fields_ = [("accel", c.c_float * 3), ("gyro", c.c_float * 3), ("temp", c.c_float)]


class Magnetic(c.Structure):
    _fields_ = [("timestamp", c.c_uint32), ("sequence", c.c_uint32), ("flags", c.c_uint32),
                ("raw", c.c_float*3), ("field", c.c_float*3), ("reference", c.c_float)]


class Estimate(c.Structure):
    _fields_ = [(name, c.c_uint32) for name in ["timestamp", "sequence", "state", "error", "flags"]] + [
        ("q", c.c_float * 4), ("euler", c.c_float * 3), ("bias", c.c_float * 3),
        ("rates", c.c_float * 3), ("accel", c.c_float * 3)] + [
        (name, c.c_uint32) for name in ["inputs", "predictions", "corrections", "publications",
                                      "cal_samples", "cal_restarts", "acc_rejects", "dt_faults", "gaps",
                                      "heading_state", "mag_samples", "mag_corrections", "mag_rejections",
                                      "mag_stale_events", "mag_timestamp", "mag_sequence"]] + [
        ("mag_body", c.c_float*3), ("mag_norm", c.c_float), ("mag_innovation", c.c_float),
        ("mag_bias", c.c_float*3)]


def specific_force(q):
    w, x, y, z = q
    return (-G * 2 * (x*z-w*y), -G * 2 * (w*x+y*z), -G * (1-2*(x*x+y*y)))


def body_to_sensor(body):
    return (-body[1], body[0], body[2])


class AttitudeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if not compiler:
            raise RuntimeError("Native gcc required for estimator validation")
        cls.directory = tempfile.TemporaryDirectory()
        library = Path(cls.directory.name) / "attitude.dll"
        build = subprocess.run([compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-shared",
                                "-I", str(ROOT / "Core/Inc"),
                                str(ROOT / "Core/Src/attitude_estimator.c"), "-lm", "-o", str(library)],
                               capture_output=True, text=True, timeout=30)
        if build.returncode:
            raise AssertionError(build.stdout + build.stderr)
        cls.lib = c.CDLL(str(library))
        cls.lib.AttitudeEstimator_Consume.argtypes = [c.POINTER(Sample), c.POINTER(Measurement)]
        cls.lib.AttitudeEstimator_GetEstimate.argtypes = [c.POINTER(Estimate)]
        cls.lib.AttitudeEstimator_Service.argtypes = [c.c_uint32, c.c_bool]
        cls.lib.AttitudeEstimator_SetMagneticSample.argtypes = [c.POINTER(Magnetic)]

    @classmethod
    def tearDownClass(cls):
        # Windows keeps a loaded DLL locked; release it before deleting the fixture.
        import _ctypes
        _ctypes.FreeLibrary(cls.lib._handle)
        cls.directory.cleanup()

    def setUp(self):
        self.lib.AttitudeEstimator_Init()
        self.time = 0
        self.seq = 0
        self.had_sent = False
        self.mag_seq = 0

    def estimate(self):
        result = Estimate()
        self.lib.AttitudeEstimator_GetEstimate(c.byref(result))
        return result

    def feed(self, gyro=(0, 0, 0), accel=(0, 0, -G), dt=1000, flags=None):
        self.time = (self.time + dt) & 0xffffffff
        self.seq = (self.seq + 1) & 0xffffffff
        sample = Sample(self.time, self.seq, dt if self.had_sent else 0)
        sample.flags = (0 if self.had_sent else 1) if flags is None else flags
        self.had_sent = True
        measured = Measurement()
        measured.accel[:] = body_to_sensor(accel)
        measured.gyro[:] = body_to_sensor(gyro)
        self.lib.AttitudeEstimator_Consume(c.byref(sample), c.byref(measured))
        return sample, measured

    def calibrate(self, bias=(0, 0, 0), accel=(0, 0, -G)):
        for _ in range(2000):
            self.feed(bias, accel)
        result = self.estimate()
        self.assertEqual((result.state, result.error, result.flags & 7), (2, 0, 7))
        return result

    def assert_rotation(self, expected, degrees=1.0):
        result = self.estimate()
        self.assertEqual((result.state, result.error), (2, 0))
        self.assertAlmostEqual(sum(x*x for x in result.q), 1, places=5)
        dot = min(1, abs(sum(a*b for a, b in zip(result.q, expected))))
        self.assertLess(math.degrees(2*math.acos(dot)), degrees)

    def magnetic(self, field=(25,0,40), calibrated=True):
        self.mag_seq += 1
        sample = Magnetic((self.time+1000)&0xffffffff, self.mag_seq, 3 if calibrated else 1)
        sample.raw[:] = sample.field[:] = field
        sample.reference = math.sqrt(25**2+40**2)
        self.lib.AttitudeEstimator_SetMagneticSample(c.byref(sample))

    def aided(self, count=1000, field=(25,0,40), calibrated=True, gyro=(0,0,0), accel=(0,0,-G)):
        for i in range(count):
            if i % 10 == 0: self.magnetic(field, calibrated)
            self.feed(gyro,accel)

    def test_magnetic_initial_heading_and_tilt_compensation(self):
        for roll,pitch,yaw in [(0,0,90),(30,20,110),(-40,-25,-135)]:
            with self.subTest(roll=roll,pitch=pitch,yaw=yaw):
                self.setUp()
                r,p,y=[math.radians(v)/2 for v in (roll,pitch,yaw)]
                q=(math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y),
                   math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
                   math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
                   math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y))
                w,x,z2,z=q
                matrix=((1-2*(z2*z2+z*z),2*(x*z2-w*z),2*(x*z+w*z2)),
                        (2*(x*z2+w*z),1-2*(x*x+z*z),2*(z2*z-w*x)),
                        (2*(x*z-w*z2),2*(w*x+z2*z),1-2*(x*x+z2*z2)))
                body=tuple(matrix[0][i]*25+matrix[2][i]*40 for i in range(3))
                accel=specific_force(q)
                self.calibrate(accel=accel)
                self.aided(field=body,accel=accel)
                self.assert_rotation(q,0.1)
                self.assertEqual(self.estimate().flags & 112,112)
                self.assertEqual(self.estimate().flags & 2,0)

    def test_magnetic_constraint_and_online_bias_stop_unbounded_yaw_drift(self):
        self.calibrate()
        self.aided()
        self.aided(count=60000,gyro=(0,0,0.01))
        result=self.estimate()
        self.assertLess(abs(math.degrees(result.euler[2])),1.0)
        self.assertGreater(result.mag_bias[2],0.003)
        self.assertLess(result.mag_bias[2],0.011)
        self.assertEqual(result.heading_state,2)

    def test_magnetic_disturbance_rejected_without_tilting_or_learning(self):
        self.calibrate(); self.aided()
        bias=list(self.estimate().mag_bias)
        for field in [(0,-25,40),(150,0,0),(float('nan'),0,40)]:
            self.aided(count=500,field=field)
            result=self.estimate()
            self.assertEqual(result.flags & 65,1)
            self.assertEqual(result.heading_state,3)
            self.assertEqual(list(result.mag_bias),bias)
            self.assert_rotation((1,0,0,0),0.1)
            self.aided(count=500)
            self.assertTrue(self.estimate().flags & 64)

    def test_magnetic_stale_and_reacquisition(self):
        self.calibrate(); self.aided()
        for _ in range(100): self.feed()
        result=self.estimate()
        self.assertEqual((result.flags & 65,result.heading_state),(1,4))
        self.aided(count=100)
        self.assertFalse(self.estimate().flags & 64)
        self.aided(count=200)
        self.assertTrue(self.estimate().flags & 64)

    def test_uncalibrated_magnetic_aiding_is_not_validated_heading(self):
        self.calibrate(); self.aided(calibrated=False)
        result=self.estimate()
        self.assertTrue(result.flags & 16)
        self.assertFalse(result.flags & 64)
        self.assertEqual(result.heading_state,1)
        self.assertEqual(list(result.mag_bias),[0,0,0])

    def test_vertical_field_and_repeated_observation_cannot_align_heading(self):
        self.calibrate(); self.aided(field=(0,0,45))
        self.assertFalse(self.estimate().flags & 64)
        self.setUp(); self.calibrate(); self.magnetic()
        for _ in range(40): self.feed()
        result=self.estimate()
        self.assertEqual(result.mag_samples,1)
        self.assertFalse(result.flags & 64)

    def test_frame_mapping_and_in_place(self):
        self.lib.AttitudeEstimator_MapToBody.argtypes = [c.POINTER(c.c_float), c.POINTER(c.c_float)]
        vector = (c.c_float * 3)(1, 2, 3)
        self.lib.AttitudeEstimator_MapToBody(vector, vector)
        self.assertEqual(list(vector), [2, -1, 3])

    def test_startup_bias_and_static_stability(self):
        bias = (0.02, -0.04, 0.01)
        result = self.calibrate(bias)
        for actual, expected in zip(result.bias, bias):
            self.assertAlmostEqual(actual, expected, places=6)
        for _ in range(10000):
            self.feed(bias)
        result = self.estimate()
        self.assert_rotation((1, 0, 0, 0), 0.1)
        self.assertEqual((result.predictions, result.corrections, result.publications), (10000, 2000, 2001))

    def test_arbitrary_initial_tilt(self):
        for roll, pitch in [(30, 20), (-60, -35), (180, 0), (0, 89)]:
            with self.subTest(roll=roll, pitch=pitch):
                self.setUp()
                r, p = math.radians(roll)/2, math.radians(pitch)/2
                expected = (math.cos(r)*math.cos(p), math.sin(r)*math.cos(p),
                            math.cos(r)*math.sin(p), -math.sin(r)*math.sin(p))
                self.calibrate(accel=specific_force(expected))
                self.assert_rotation(expected, 0.1)

    def test_known_single_axis_rotations(self):
        for axis in range(3):
            for direction in [-1, 1]:
                with self.subTest(axis=axis, direction=direction):
                    self.setUp()
                    self.calibrate()
                    rate = direction * math.pi/2
                    for i in range(1, 1001):
                        q = [math.cos(rate*i*0.001/2), 0, 0, 0]
                        q[axis+1] = math.sin(rate*i*0.001/2)
                        gyro = [0, 0, 0]; gyro[axis] = rate
                        self.feed(gyro, specific_force(q))
                    self.assert_rotation(q, 1.0)

    def test_coupled_rotation_and_timestamp_jitter(self):
        self.calibrate()
        rate = (0.6, -0.8, 0.3)
        magnitude = math.sqrt(sum(v*v for v in rate))
        elapsed = 0
        for i in range(2000):
            dt = 700 if i % 2 else 1300
            elapsed += dt
            angle = magnitude*elapsed*1e-6/2
            expected = [math.cos(angle)] + [v/magnitude*math.sin(angle) for v in rate]
            self.feed(rate, specific_force(expected), dt=dt)
        self.assert_rotation(expected, 1.0)

    def test_every_gyro_sample_is_integrated(self):
        self.calibrate()
        for i in range(1000):
            self.feed((0, 0, 2 if i % 5 == 0 else 0), (0, 0, 0))
        self.assert_rotation((math.cos(0.2), 0, 0, math.sin(0.2)), 0.1)

    def test_publication_phase_with_realistic_fast_sensor_clock(self):
        self.calibrate()
        for _ in range(10000): self.feed(dt=999)
        result = self.estimate()
        self.assertEqual(result.predictions, 10000)
        self.assertEqual(result.corrections, 1998)
        self.assertEqual(result.publications, 1999)

    def test_movement_restarts_startup_window(self):
        for _ in range(1000): self.feed()
        for _ in range(50): self.feed((0.2, 0, 0))
        result = self.estimate()
        self.assertEqual((result.state, result.cal_samples, result.cal_restarts), (1, 0, 1))
        self.calibrate()

    def test_variable_slow_motion_fails_variance_gate(self):
        for i in range(2000): self.feed((0.02 * math.sin(i*0.03), 0, 0))
        result = self.estimate()
        self.assertEqual((result.state, result.cal_samples, result.cal_restarts), (1, 0, 1))
        self.assertEqual(result.flags & 1, 0)

    def test_acceleration_and_freefall_do_not_create_tilt(self):
        self.calibrate()
        for accel in [(G*2, 0, -G), (0, 0, 0)]:
            for _ in range(1000): self.feed(accel=accel)
            self.assert_rotation((1, 0, 0, 0), 0.1)
            self.assertFalse(self.estimate().flags & 8)
        self.assertGreater(self.estimate().acc_rejects, 300)

    def test_gravity_feedback_corrects_roll_disturbance(self):
        self.calibrate()
        for _ in range(100): self.feed((1, 0, 0), (0, 0, 0))
        for _ in range(5000): self.feed()
        self.assert_rotation((1, 0, 0, 0), 0.1)

    def test_relative_yaw_is_not_artificially_corrected(self):
        self.calibrate()
        for _ in range(10000): self.feed((0, 0, 0.01))
        self.assert_rotation((math.cos(0.05), 0, 0, math.sin(0.05)), 0.1)
        self.assertTrue(self.estimate().flags & 2)

    def test_timestamp_and_sequence_wrap(self):
        self.calibrate()
        # Start a fresh calibration across both 32-bit wrap boundaries.
        self.setUp(); self.time = 0xfffff000; self.seq = 0xfffffff0
        self.calibrate()
        self.assertEqual(self.estimate().dt_faults, 0)

    def test_gaps_and_duplicate_samples_latch_fault(self):
        for mode in ["sequence", "flag", "duplicate"]:
            with self.subTest(mode=mode):
                self.setUp(); self.calibrate()
                if mode == "sequence": self.seq += 1
                self.feed(flags=2 if mode == "flag" else 0, dt=0 if mode == "duplicate" else 1000)
                result = self.estimate()
                self.assertEqual((result.state, result.error), (3, 2 if mode == "duplicate" else 3))
                self.assertFalse(result.flags & 1)
                for _ in range(2100): self.feed()
                self.assertEqual(self.estimate().state, 3)

    def test_timing_mismatch_and_long_interval(self):
        for mode in ["long", "metadata"]:
            with self.subTest(mode=mode):
                self.setUp(); self.calibrate()
                if mode == "long": self.feed(dt=2000)
                else:
                    sample, measured = self.feed()
                    sample.timestamp += 1000; sample.sequence += 1; sample.dt = 999
                    self.lib.AttitudeEstimator_Consume(c.byref(sample), c.byref(measured))
                self.assertEqual((self.estimate().state, self.estimate().error), (3, 2))

    def test_stale_or_faulted_sensor_invalidates_attitude(self):
        for healthy in [True, False]:
            with self.subTest(healthy=healthy):
                self.setUp(); self.calibrate()
                self.lib.AttitudeEstimator_Service((self.time+6000) & 0xffffffff, healthy)
                self.assertEqual(self.estimate().error, 2 if healthy else 4)
                self.assertFalse(self.estimate().flags & 1)

    def test_nonfinite_saturation_and_null_input(self):
        for mode in ["nan", "infinity", "saturated", "null"]:
            with self.subTest(mode=mode):
                self.setUp(); self.calibrate()
                if mode == "null": self.lib.AttitudeEstimator_Consume(None, None)
                else:
                    sample, measured = self.feed()
                    if mode == "nan": measured.gyro[0] = float('nan')
                    if mode == "infinity": measured.accel[1] = float('inf')
                    if mode == "saturated": sample.gyro[1] = 32767
                    self.lib.AttitudeEstimator_Consume(c.byref(sample), c.byref(measured))
                self.assertEqual((self.estimate().state, self.estimate().error), (3, 1))


if __name__ == '__main__':
    unittest.main()
