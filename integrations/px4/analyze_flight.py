#!/usr/bin/env python3
"""Independently check logged SIH truth against estimator and flight state."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog


def columns(DATA, NAMES):
    return np.column_stack([DATA[NAME] for NAME in NAMES])


def interpolate(SOURCE, NAMES, TIMESTAMPS):
    T = SOURCE.get("timestamp_sample", SOURCE["timestamp"])
    return np.column_stack([np.interp(TIMESTAMPS, T, SOURCE[NAME]) for NAME in NAMES])


def rms(VECTOR):
    return float(np.sqrt(np.mean(np.sum(VECTOR**2, axis=1))))


def analyze(DIRECTORY):
    FLIGHT = json.loads((DIRECTORY / "flight.json").read_text())
    if FLIGHT["result"] != "flight_pass_pending_ulog_analysis":
        raise RuntimeError("Live flight checks did not pass")
    PATHS = [DIRECTORY / PATH for PATH in FLIGHT["ulogs"]]
    if len(PATHS) != 1:
        raise RuntimeError(f"Expected one ULog, found {len(PATHS)}")
    LOG = ULog(str(PATHS[0]))
    LOCAL = LOG.get_dataset("vehicle_local_position").data
    TRUTH = LOG.get_dataset("vehicle_local_position_groundtruth").data
    ATTITUDE = LOG.get_dataset("vehicle_attitude").data
    TRUE_ATTITUDE = LOG.get_dataset("vehicle_attitude_groundtruth").data
    STATUS = LOG.get_dataset("vehicle_status").data
    LAND = LOG.get_dataset("vehicle_land_detected").data
    ESTIMATOR = LOG.get_dataset("estimator_status").data
    ODOMETRY = next((D.data for D in LOG.data_list if D.name == "vehicle_odometry"), None)
    AIRBORNE_TIMES = LAND["timestamp"][LAND["landed"] == 0]
    if len(AIRBORNE_TIMES) == 0:
        raise RuntimeError("Land detector never reported airborne")
    START, END = AIRBORNE_TIMES[0], AIRBORNE_TIMES[-1]
    T = LOCAL["timestamp_sample"]
    MASK = (T >= START) & (T <= END)
    T = T[MASK]
    P_EST = columns(LOCAL, ["x", "y", "z"])[MASK]
    V_EST = columns(LOCAL, ["vx", "vy", "vz"])[MASK]
    P_TRUE = interpolate(TRUTH, ["x", "y", "z"], T)
    V_TRUE = interpolate(TRUTH, ["vx", "vy", "vz"], T)
    # Account for independently initialized local origins. Translation comes
    # from the logged geographic references, never fitted to flight errors.
    RADIUS = 6371000.
    ORIGIN_OFFSET = np.array([
        np.deg2rad(float(LOCAL["ref_lat"][0] - TRUTH["ref_lat"][0])) * RADIUS,
        np.deg2rad(float(LOCAL["ref_lon"][0] - TRUTH["ref_lon"][0])) * RADIUS
        * np.cos(np.deg2rad(float(TRUTH["ref_lat"][0]))),
        float(TRUTH["ref_alt"][0] - LOCAL["ref_alt"][0]),
    ])
    P_EST_IN_TRUTH_FRAME = P_EST + ORIGIN_OFFSET
    POSITION_ERROR = P_EST_IN_TRUTH_FRAME - P_TRUE
    VELOCITY_ERROR = V_EST - V_TRUE
    QT = ATTITUDE["timestamp_sample"]
    QMASK = (QT >= START) & (QT <= END)
    Q_EST = columns(ATTITUDE, [f"q[{I}]" for I in range(4)])[QMASK]
    Q_TRUTH = interpolate(TRUE_ATTITUDE, [f"q[{I}]" for I in range(4)], QT[QMASK])
    Q_TRUTH /= np.linalg.norm(Q_TRUTH, axis=1)[:, None]
    Q_NORM_ERROR = np.abs(np.linalg.norm(Q_EST, axis=1) - 1.)
    Q_EST /= np.linalg.norm(Q_EST, axis=1)[:, None]
    ANGLE_ERROR = np.rad2deg(2. * np.arccos(np.clip(np.abs(np.sum(Q_EST * Q_TRUTH, axis=1)), 0., 1.)))
    SMASK = (STATUS["timestamp"] >= START) & (STATUS["timestamp"] <= END)
    EMASK = (ESTIMATOR["timestamp"] >= START) & (ESTIMATOR["timestamp"] <= END)
    FINITE = bool(np.isfinite(P_EST).all() and np.isfinite(V_EST).all() and np.isfinite(Q_EST).all())
    VALID = np.logical_and.reduce([LOCAL[NAME][MASK] for NAME in ["xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control"]])
    COVARIANCE_DIAGONALS = (columns(ODOMETRY, [f"{GROUP}_variance[{AXIS}]"
                                              for GROUP in ["position", "orientation", "velocity"] for AXIS in range(3)])
                            if ODOMETRY is not None else np.array([]))
    VALUES = {
        "airborne_duration_s": float((END - START) * 1e-6),
        "max_truth_height_m": float(-np.min(P_TRUE[:, 2])),
        "truth_north_span_m": float(np.ptp(P_TRUE[:, 0])),
        "truth_east_span_m": float(np.ptp(P_TRUE[:, 1])),
        "position_3d_rmse_m": rms(POSITION_ERROR),
        "position_3d_max_error_m": float(np.max(np.linalg.norm(POSITION_ERROR, axis=1))),
        "velocity_3d_rmse_m_s": rms(VELOCITY_ERROR),
        "attitude_rmse_deg": float(np.sqrt(np.mean(ANGLE_ERROR**2))),
        "attitude_max_error_deg": float(np.max(ANGLE_ERROR)),
        "quaternion_max_norm_error": float(np.max(Q_NORM_ERROR)),
        "valid_estimate_fraction": float(np.mean(VALID)),
        "final_truth_height_m": float(-TRUTH["z"][-1]),
        "ulog_dropout_count": len(LOG.dropouts),
        "ulog_dropout_total_s": sum(DROP.duration for DROP in LOG.dropouts) * 1e-3,
        "origin_translation_to_truth_ned_m": ORIGIN_OFFSET.tolist(),
        "logged_topics": sorted({DATASET.name for DATASET in LOG.data_list}),
    }
    CHECKS = {
        "normal_arm_and_all_waypoints": len(FLIGHT["phases"]) == 5 and FLIGHT["normal_arming"],
        "no_ekf2_output": not any(DATASET.name in ("ekf2_timestamps", "estimator_selector_status") for DATASET in LOG.data_list),
        "finite_state": FINITE,
        "finite_nonnegative_published_variances": bool(COVARIANCE_DIAGONALS.size > 0 and np.isfinite(COVARIANCE_DIAGONALS).all() and (COVARIANCE_DIAGONALS >= 0).all()),
        "airborne_at_least_50_s": VALUES["airborne_duration_s"] >= 50.,
        "reached_5_m": 4. <= VALUES["max_truth_height_m"] <= 7.,
        "flew_north_and_east": VALUES["truth_north_span_m"] >= 6. and VALUES["truth_east_span_m"] >= 6.,
        "position_rmse_below_1_5_m": VALUES["position_3d_rmse_m"] < 1.5,
        "velocity_rmse_below_0_5_m_s": VALUES["velocity_3d_rmse_m_s"] < 0.5,
        "attitude_rmse_below_5_deg": VALUES["attitude_rmse_deg"] < 5.,
        "attitude_max_below_15_deg": VALUES["attitude_max_error_deg"] < 15.,
        "unit_quaternion": VALUES["quaternion_max_norm_error"] < 1e-5,
        "valid_estimate_through_flight": VALUES["valid_estimate_fraction"] > 0.99,
        "no_numerical_fault_flags": bool((ESTIMATOR["filter_fault_flags"][EMASK] == 0).all()),
        "no_failsafe_in_air": bool((STATUS["failsafe"][SMASK] == 0).all()),
        "landed_near_ground": abs(VALUES["final_truth_height_m"]) < 0.5,
        "auto_disarmed": any("automatically disarmed" in EVENT["event"] for EVENT in FLIGHT["events"]),
        "ulog_dropout_below_1_percent": VALUES["ulog_dropout_total_s"] < 0.01 * VALUES["airborne_duration_s"],
    }
    REPORT = {"result": "pass" if all(CHECKS.values()) else "fail", "checks": CHECKS, "metrics": VALUES,
              "ulog": str(PATHS[0].relative_to(DIRECTORY)),
              "scope": "PX4 v1.16.2 SIH quadx, default simulated sensor noise, single IMU/GNSS/magnetometer/barometer. No hardware or fault-tolerance qualification."}
    (DIRECTORY / "analysis.json").write_text(json.dumps(REPORT, indent=2, allow_nan=False))
    FIG, AXES = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    AXES[0, 0].plot(P_TRUE[:, 1], P_TRUE[:, 0], label="SIH truth")
    AXES[0, 0].plot(P_EST_IN_TRUTH_FRAME[:, 1], P_EST_IN_TRUTH_FRAME[:, 0], label="formal-eskf", alpha=.8)
    AXES[0, 0].set(xlabel="East (m)", ylabel="North (m)", title="Closed-loop square flight", aspect="equal")
    AXES[0, 0].legend()
    SECONDS = (T - START) * 1e-6
    AXES[0, 1].plot(SECONDS, -P_TRUE[:, 2], label="SIH truth")
    AXES[0, 1].plot(SECONDS, -P_EST_IN_TRUTH_FRAME[:, 2], label="formal-eskf", alpha=.8)
    AXES[0, 1].set(xlabel="Airborne time (s)", ylabel="Height (m)", title="Takeoff, hover, auto-land")
    AXES[0, 1].legend()
    for AXIS, NAME in enumerate(["North", "East", "Down"]):
        AXES[1, 0].plot(SECONDS, POSITION_ERROR[:, AXIS], label=NAME)
    AXES[1, 0].set(xlabel="Airborne time (s)", ylabel="Estimate - truth (m)", title="Position estimation error")
    AXES[1, 0].legend()
    AXES[1, 1].plot((QT[QMASK] - START) * 1e-6, ANGLE_ERROR)
    AXES[1, 1].set(xlabel="Airborne time (s)", ylabel="Quaternion angle error (deg)", title="Attitude estimation error")
    for AX in AXES.flat:
        AX.grid(alpha=.3)
    FIG.suptitle(f"formal-eskf replaces EKF2 | PX4 SIH | {REPORT['result'].upper()}")
    FIG.savefig(DIRECTORY / "flight.png", dpi=160)
    plt.close(FIG)
    print(json.dumps({"result": REPORT["result"], "checks": CHECKS,
                      "metrics": {KEY: VALUE for KEY, VALUE in VALUES.items() if KEY != "logged_topics"}}, indent=2))
    return all(CHECKS.values())


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("directory", type=Path)
    ARGS = PARSER.parse_args()
    raise SystemExit(0 if analyze(ARGS.directory) else 1)
