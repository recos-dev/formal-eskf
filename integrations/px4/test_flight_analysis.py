#!/usr/bin/env python3
"""Regression checks for simulator ground-truth reference alignment."""
import unittest

import numpy as np

from analyze_flight import flight_reference


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.data = {
            "timestamp": np.array([1, 2, 3, 4, 5]),
            "ref_timestamp": np.array([0, 2, 2, 2, 2]),
            "xy_global": np.array([0, 1, 1, 1, 1]),
            "z_global": np.array([0, 1, 1, 1, 1]),
            "ref_lat": np.array([np.nan, 47., 47., 47., 47.]),
            "ref_lon": np.array([np.nan, 8., 8., 8., 8.]),
            "ref_alt": np.array([np.nan, 488., 488., 488., 488.]),
        }

    def test_gazebo_startup_without_gnss_reference(self):
        np.testing.assert_array_equal(flight_reference(self.data, 3, 5), [47., 8., 488.])

    def test_missing_preflight_reference_is_not_fitted_from_flight(self):
        self.data["xy_global"][:3] = 0
        with self.assertRaisesRegex(RuntimeError, "before takeoff"):
            flight_reference(self.data, 3, 5)

    def test_reference_initialized_at_simulation_time_zero(self):
        self.data["ref_timestamp"][:] = 0
        np.testing.assert_array_equal(flight_reference(self.data, 3, 5), [47., 8., 488.])

    def test_origin_change_during_flight_is_rejected(self):
        self.data["ref_alt"][4] += 1.
        with self.assertRaisesRegex(RuntimeError, "changing geographic"):
            flight_reference(self.data, 3, 5)

    def test_invalid_reference_during_flight_is_rejected(self):
        self.data["ref_lat"][4] = np.nan
        with self.assertRaisesRegex(RuntimeError, "geographic reference during flight"):
            flight_reference(self.data, 3, 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
