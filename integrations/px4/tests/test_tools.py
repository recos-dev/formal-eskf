#!/usr/bin/env python3
"""Offline regression checks for flight acceptance, fault checks and tool entry points."""

import json
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.dont_write_bytecode = True
INTEGRATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(INTEGRATION_DIR / "scripts"))

import numpy as np
from analyze_flight import acceptance_checks, load_flight
from sitl_flight import Flight


class AcceptanceTests(unittest.TestCase):
    def setUp(SELF):
        SELF.FLIGHT = {"normal_arming": True, "phases": [None] * 5,
                       "events": [{"event": "Auto-land completed and automatically disarmed"}]}
        SELF.LOG = SimpleNamespace(data_list=[SimpleNamespace(name="vehicle_local_position")])
        SELF.METRICS = {
            "airborne_duration_s": 60., "max_truth_height_m": 5.,
            "truth_north_span_m": 8., "truth_east_span_m": 8.,
            "position_3d_rmse_m": 0.3, "velocity_3d_rmse_m_s": 0.1,
            "attitude_rmse_deg": 1., "attitude_max_error_deg": 2.,
            "quaternion_max_norm_error": 1e-8, "valid_estimate_fraction": 1.,
            "final_truth_height_m": 0., "ulog_dropout_total_s": 0.,
        }

    def checks(SELF, VARIANCES=None):
        return acceptance_checks(SELF.FLIGHT, SELF.LOG, SELF.METRICS, True,
                                 np.ones(9) if VARIANCES is None else VARIANCES,
                                 np.zeros(4), np.zeros(4))

    def test_nominal_flight_and_landing_failure(SELF):
        SELF.assertTrue(all(SELF.checks().values()))
        SELF.METRICS["final_truth_height_m"] = -2.
        FAILED = [NAME for NAME, PASSED in SELF.checks().items() if not PASSED]
        SELF.assertEqual(FAILED, ["landed_near_ground"])

    def test_accuracy_boundaries_are_not_relaxed(SELF):
        for METRIC, LIMIT, CHECK in (
            ("position_3d_rmse_m", 1.5, "position_rmse_below_1_5_m"),
            ("velocity_3d_rmse_m_s", 0.5, "velocity_rmse_below_0_5_m_s"),
            ("attitude_rmse_deg", 5., "attitude_rmse_below_5_deg"),
        ):
            with SELF.subTest(METRIC=METRIC):
                SELF.METRICS[METRIC] = LIMIT
                SELF.assertFalse(SELF.checks()[CHECK])

    def test_missing_or_invalid_covariance_fails(SELF):
        for VALUES in (np.array([]), np.array([np.nan]), np.array([-0.1])):
            SELF.assertFalse(SELF.checks(VALUES)["finite_nonnegative_published_variances"])

    def test_ekf2_output_is_not_accepted(SELF):
        SELF.LOG.data_list.append(SimpleNamespace(name="ekf2_timestamps"))
        SELF.assertFalse(SELF.checks()["no_ekf2_output"])

    def test_failed_flight_cannot_enter_analysis(SELF):
        with tempfile.TemporaryDirectory() as TEMP:
            ROOT = Path(TEMP)
            (ROOT / "flight.json").write_text(json.dumps({"result": "fail", "ulogs": []}))
            with SELF.assertRaisesRegex(RuntimeError, "Live flight checks"):
                load_flight(ROOT)


def listener(TEXT):
    return {"output": TEXT}


class FaultTests(unittest.TestCase):
    def test_disabled_gravity_must_not_fuse(SELF):
        FLAGS = listener("cs_gravity_vector: False")
        HEIGHT = listener("gravity: enabled=0 tilt=1 active=0 samples=10 fused=0")
        SELF.assertTrue(Flight.check_gravity("gnss_timeout", HEIGHT, FLAGS, False))
        HEIGHT["output"] = HEIGHT["output"].replace("fused=0", "fused=1")
        SELF.assertFalse(Flight.check_gravity("gnss_timeout", HEIGHT, FLAGS, False))

    def test_gnss_recovery_counts_wrap_once(SELF):
        LOCAL = listener("xy_reset_counter: 0\nvxy_reset_counter: 0")
        SELF.assertTrue(Flight.check_gnss("gnss_restart", LOCAL, listener(""), listener(""), 255, 255))
        SELF.assertFalse(Flight.check_gnss("gnss_restart", LOCAL, listener(""), listener(""), 254, 255))

    def test_magnetic_reset_agrees_across_topics(SELF):
        LOCAL = listener("heading_reset_counter: 1")
        STATUS = listener("reset_count_quat: 1")
        ATTITUDE = listener("quat_reset_counter: 1")
        FLAGS = listener("cs_mag_3d: False\ncs_mag: True\ncs_mag_hdg: True\n"
                         "cs_yaw_align: True\ncs_mag_heading_consistent: True")
        SELF.assertTrue(Flight.check_magnetic("mag_unaided_restart", LOCAL, STATUS, FLAGS, ATTITUDE, 0))
        ATTITUDE["output"] = "quat_reset_counter: 0"
        SELF.assertFalse(Flight.check_magnetic("mag_unaided_restart", LOCAL, STATUS, FLAGS, ATTITUDE, 0))

    def test_height_reset_preserves_ned_altitude_sign(SELF):
        LOCAL = listener("z_reset_counter: 1\ndelta_z: -2.0")
        STATUS = listener("reset_count_pod_d: 1")
        GLOBAL = listener("alt_reset_counter: 1\ndelta_alt: 2.0")
        SELF.assertTrue(Flight.check_height("gnss_restart", LOCAL, STATUS, GLOBAL, listener(""), 0))
        GLOBAL["output"] = GLOBAL["output"].replace("2.0", "-2.0")
        SELF.assertFalse(Flight.check_height("gnss_restart", LOCAL, STATUS, GLOBAL, listener(""), 0))


class CommandTests(unittest.TestCase):
    def test_shutdown_reaps_own_process_when_cli_times_out(SELF):
        RUN = object.__new__(Flight)
        RUN.process = Mock(pid=12345)
        RUN.process.poll.return_value = None
        RUN.process.wait.side_effect = [subprocess.TimeoutExpired("px4", 10),
                                       subprocess.TimeoutExpired("px4", 5), 0]
        RUN.cli = Mock(side_effect=subprocess.TimeoutExpired("shutdown", 10))
        RESULT = {}
        with patch("sitl_flight.os.killpg") as KILL:
            RUN.shutdown(RESULT)
        SELF.assertEqual([CALL.args for CALL in KILL.call_args_list],
                         [(12345, signal.SIGTERM), (12345, signal.SIGKILL)])
        SELF.assertIn("shutdown_error", RESULT)

    def test_shutdown_leaves_finished_process_alone(SELF):
        RUN = object.__new__(Flight)
        RUN.process = Mock()
        RUN.process.poll.return_value = 0
        RUN.cli = Mock()
        RUN.shutdown({})
        RUN.cli.assert_not_called()
        RUN.process.wait.assert_not_called()

    def test_shell_help_and_invalid_usage(SELF):
        for NAME in ("scripts/setup.sh", "tests/test_core.sh"):
            SCRIPT = str(INTEGRATION_DIR / NAME)
            SELF.assertEqual(subprocess.run([SCRIPT, "--help"], capture_output=True).returncode, 0)
            SELF.assertNotEqual(subprocess.run([SCRIPT], capture_output=True).returncode, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
