#!/usr/bin/env python3
"""Replay a legacy PX4 ULog through the prebuilt C++ INS example.

Sensor inputs are sensor_combined, vehicle_gps_position and vehicle_magnetometer;
vehicle_land_detected supplies optional at_rest metadata.
PX4 estimator outputs are not inputs. Magnetic measurements initialize attitude
and provide lifecycle-controlled, gated corrections against a fixed seed field.
Gravity correction requires complete clipping metadata and valid fusion conditions.
GNSS quality qualifies lifecycle, position/velocity resets and independent velocity/position/height gates.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from replay_common import (IMU_HEADER, INITIAL_HEADER, NAVIGATION_HEADER, PROGRESS_FORMAT,
                           dot, ecef, ned_basis, plot_replay, write_csv)


def topic(ULOG, NAME):
    MATCHES = [DATA.data for DATA in ULOG.data_list if DATA.name == NAME and DATA.multi_id == 0]
    if len(MATCHES) != 1:
        raise ValueError(f"expected one {NAME} instance 0")
    return MATCHES[0]


def timestamps(DATA):
    TIMES = [int(TIME) for TIME in DATA["timestamp"]]
    if not TIMES or TIMES[0] <= 0 or any(B <= A for A, B in zip(TIMES, TIMES[1:])):
        raise ValueError("topic timestamps must be nonzero and strictly increasing")
    return TIMES


def finite(VALUES):
    return all(math.isfinite(float(VALUE)) for VALUE in VALUES)


def xyz(DATA, FIELD, INDEX):
    return [float(DATA[f"{FIELD}[{AXIS}]"][INDEX]) for AXIS in range(3)]


def initial_quaternion(ACCEL, MAG, DECLINATION):
    """Static tilt alignment, followed by tilt-compensated magnetic heading.

    ACCEL is specific force in FRD; DECLINATION is east-positive, in radians.
    Returns scalar-first Hamilton q_nb. This is an initial guess, not a sensor
    calibration or an independent attitude reference.
    """
    NORM = math.sqrt(sum(VALUE * VALUE for VALUE in ACCEL))
    if not finite([*ACCEL, *MAG, DECLINATION]) or not 5.0 < NORM < 15.0:
        raise ValueError("unsafe accelerometer/magnetometer initialization")
    ROLL = math.atan2(-ACCEL[1], -ACCEL[2])
    PITCH = math.asin(max(-1.0, min(1.0, ACCEL[0] / NORM)))
    CR, SR, CP, SP = math.cos(ROLL), math.sin(ROLL), math.cos(PITCH), math.sin(PITCH)
    MX = CP * MAG[0] + SP * (SR * MAG[1] + CR * MAG[2])
    MY = CR * MAG[1] - SR * MAG[2]
    if math.hypot(MX, MY) < 1e-6:
        raise ValueError("magnetic horizontal component is too small")
    YAW = DECLINATION - math.atan2(MY, MX)
    CR, SR = math.cos(ROLL / 2), math.sin(ROLL / 2)
    CP, SP = math.cos(PITCH / 2), math.sin(PITCH / 2)
    CY, SY = math.cos(YAW / 2), math.sin(YAW / 2)
    return [CR * CP * CY + SR * SP * SY, SR * CP * CY - CR * SP * SY,
            CR * SP * CY + SR * CP * SY, CR * CP * SY - SR * SP * CY]


def magnetic_reference(Q, FIELD, ULOG):
    """Freeze the causal seed field in NED; this is not an independent WMM reference."""
    Q0, Q1, Q2, Q3 = Q
    ROTATION = [[1 - 2 * (Q2 * Q2 + Q3 * Q3), 2 * (Q1 * Q2 - Q0 * Q3), 2 * (Q1 * Q3 + Q0 * Q2)],
                [2 * (Q1 * Q2 + Q0 * Q3), 1 - 2 * (Q1 * Q1 + Q3 * Q3), 2 * (Q2 * Q3 - Q0 * Q1)],
                [2 * (Q1 * Q3 - Q0 * Q2), 2 * (Q2 * Q3 + Q0 * Q1), 1 - 2 * (Q1 * Q1 + Q2 * Q2)]]
    # The published topic is already calibrated and body-aligned. Do not apply
    # CAL_MAG offsets again or import EKF2's learned magnetic bias estimates.
    REFERENCE = [100.0 * dot(ROW, FIELD) for ROW in ROTATION]  # Gauss -> microtesla.
    NOISE_STD = 100.0 * float(ULOG.initial_parameters["EKF2_MAG_NOISE"])
    GATE_SIGMA = float(ULOG.initial_parameters["EKF2_MAG_GATE"])
    if not finite([*REFERENCE, NOISE_STD, GATE_SIGMA]) or NOISE_STD <= 0 or GATE_SIGMA <= 0:
        raise ValueError("invalid magnetic reference/noise/gate parameters")
    return [*REFERENCE, NOISE_STD, GATE_SIGMA]


def magnetic_rows(ULOG, MAG, TIMES, START, END):
    """One calibrated measurement per message; reject known frame/source changes."""
    for NAME in ("device_id", "calibration_count"):
        if NAME in MAG:
            VALUES = {int(VALUE) for TIME, VALUE in zip(TIMES, MAG[NAME]) if TIME <= END}
            if len(VALUES) != 1:
                raise ValueError(f"magnetometer {NAME} changes require a new replay/reference")
    SELECTION = [DATA.data for DATA in ULOG.data_list
                 if DATA.name == "sensor_selection" and DATA.multi_id == 0]
    for DATA in SELECTION:
        if "mag_device_id" in DATA:
            IDS = {int(VALUE) for TIME, VALUE in zip(DATA["timestamp"], DATA["mag_device_id"])
                   if int(TIME) <= END}
            if len(IDS) > 1:
                raise ValueError("selected magnetometer changes require a new replay/reference")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME.startswith(("CAL_MAG", "SENS_BOARD", "EKF2_MAG_NOISE", "EKF2_MAG_GATE",
                                           "EKF2_MAG_DECL")):
            raise ValueError(f"changed {NAME} requires a new replay/reference")
    ROWS = []
    for INDEX, TIME in enumerate(TIMES):
        if START < TIME <= END:
            VALUES = [100.0 * VALUE for VALUE in xyz(MAG, "magnetometer_ga", INDEX)]
            if not finite(VALUES):
                raise ValueError("non-finite magnetometer reading")
            ROWS.append([TIME, *VALUES])
    if not ROWS:
        raise ValueError("no magnetometer samples after initialization")
    return ROWS


def gnss_parameters(ULOG, END):
    """Fixed settings for a sole-GNSS aiding replay; no lever-arm compensation."""
    NAMES = ("EKF2_GPS_P_GATE", "EKF2_GPS_V_GATE", "EKF2_GPS_P_NOISE", "EKF2_GPS_V_NOISE", "EKF2_NOAID_NOISE")
    POSITION_NAMES = tuple(f"EKF2_{SENSOR}_POS_{AXIS}" for SENSOR in ("GPS", "IMU") for AXIS in "XYZ")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    OFFSETS = [float(ULOG.initial_parameters[NAME]) for NAME in POSITION_NAMES]
    if not finite([*VALUES, *OFFSETS]) or min(VALUES[2:]) <= 0:
        raise ValueError("invalid GNSS noise/gate/offset parameters")
    if OFFSETS[:3] != OFFSETS[3:]:
        raise ValueError("GNSS replay requires co-located GNSS/IMU; lever-arm compensation is not implemented")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, *POSITION_NAMES):
            raise ValueError(f"changed {NAME} requires a new GNSS replay configuration")
    VALUES[:2] = [max(VALUE, 1.0) for VALUE in VALUES[:2]]
    return VALUES


def gnss_quality_parameters(ULOG, END):
    """Use recorded checks/limits, with the legacy name and fixed health-time default explicit."""
    PDOP_NAME = "EKF2_REQ_PDOP" if "EKF2_REQ_PDOP" in ULOG.initial_parameters else "EKF2_REQ_GDOP"
    NAMES = ("EKF2_GPS_CHECK", "EKF2_REQ_NSATS", PDOP_NAME, "EKF2_REQ_EPH", "EKF2_REQ_EPV",
             "EKF2_REQ_SACC", "EKF2_REQ_HDRIFT", "EKF2_REQ_VDRIFT")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    HEALTH_SECONDS = float(ULOG.initial_parameters.get("EKF2_REQ_GPS_H", 10.0))
    HEALTH_US = round(HEALTH_SECONDS * 1e6) if math.isfinite(HEALTH_SECONDS) else 0
    if (not finite(VALUES) or not VALUES[0].is_integer() or not 0 <= VALUES[0] <= 1023
            or not VALUES[1].is_integer() or not 1 <= VALUES[1] <= 255
            or min(VALUES[2:]) <= 0 or not 1 <= HEALTH_US <= 4294967295):
        raise ValueError("invalid GNSS quality parameters")
    if ("EKF2_REQ_PDOP" in ULOG.initial_parameters and "EKF2_REQ_GDOP" in ULOG.initial_parameters
            and ULOG.initial_parameters["EKF2_REQ_PDOP"] != ULOG.initial_parameters["EKF2_REQ_GDOP"]):
        raise ValueError("conflicting GNSS PDOP parameter names")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, "EKF2_REQ_PDOP", "EKF2_REQ_GDOP", "EKF2_REQ_GPS_H"):
            raise ValueError(f"changed {NAME} requires a new GNSS quality session")
    return [*VALUES, HEALTH_US]


def gps_reading(GPS, INDEX, REQUIRE_VALID=True):
    LAT = math.radians(float(GPS["lat"][INDEX]) * 1e-7)
    LON = math.radians(float(GPS["lon"][INDEX]) * 1e-7)
    ALT = float(GPS["alt_ellipsoid"][INDEX]) * 1e-3
    VELOCITY = [float(GPS[FIELD][INDEX]) for FIELD in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s")]
    ACCURACY = [float(GPS[FIELD][INDEX]) for FIELD in ("eph", "epv", "s_variance_m_s")]
    if REQUIRE_VALID and (int(GPS["fix_type"][INDEX]) < 3 or not GPS["vel_ned_valid"][INDEX]
            or not finite([LAT, LON, ALT, *VELOCITY, *ACCURACY])
            or abs(LAT) > math.pi / 2 or abs(LON) > math.pi or min(ACCURACY) <= 0):
        return None
    if not finite([LAT, LON, ALT]) or abs(LAT) > math.pi / 2 or abs(LON) > math.pi:
        raise ValueError("invalid GNSS coordinates; replay cannot project this input")
    return LAT, LON, ALT, VELOCITY, ACCURACY


def land_flags(ULOG, TIMES):
    """Causal in-air/rest metadata, unknown after 3 s; landed does not establish rest."""
    LAND = [DATA.data for DATA in ULOG.data_list
            if DATA.name == "vehicle_land_detected" and DATA.multi_id == 0]
    if len(LAND) > 1:
        raise ValueError("multiple vehicle_land_detected instance 0 topics")
    LAND = LAND[0] if LAND else None
    LAND_TIMES = timestamps(LAND) if LAND is not None else []
    LAND_INDEX = -1
    ROWS = []
    for TIME in TIMES:
        while LAND_INDEX + 1 < len(LAND_TIMES) and LAND_TIMES[LAND_INDEX + 1] <= TIME:
            LAND_INDEX += 1
        VALUES = [-1, -1]
        if LAND_INDEX >= 0 and TIME - LAND_TIMES[LAND_INDEX] < 3000000:
            for AXIS, NAME in enumerate(("landed", "at_rest")):
                if NAME in LAND:
                    VALUE = LAND[NAME][LAND_INDEX]
                    if VALUE not in (0, 1):
                        raise ValueError(f"invalid {NAME} flag")
                    VALUES[AXIS] = 1 - int(VALUE) if NAME == "landed" else int(VALUE)
        ROWS.append(VALUES)
    return ROWS


def imu_flags(ULOG, IMU, START_INDEX):
    """Missing clipping/rest data is -1, never inferred from estimator outputs."""
    TIMES = [int(TIME) for TIME in IMU["timestamp"][START_INDEX:]]
    for NAME in ("accel_calibration_count", "gyro_calibration_count"):
        if NAME in IMU and len(set(IMU[NAME][START_INDEX:])) > 1:
            raise ValueError(f"IMU {NAME} changes require a new replay/bias-learning session")
    for DATA in ULOG.data_list:
        if DATA.name == "sensor_selection" and DATA.multi_id == 0:
            for NAME in ("accel_device_id", "gyro_device_id"):
                if NAME in DATA.data:
                    IDS = {int(VALUE) for TIME, VALUE in zip(DATA.data["timestamp"], DATA.data[NAME])
                           if TIME <= TIMES[-1]}
                    if len(IDS) > 1:
                        raise ValueError(f"selected IMU {NAME} changes require a new replay/bias-learning session")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIMES[0] <= TIME <= TIMES[-1] and NAME.startswith(("CAL_ACC", "CAL_GYRO", "SENS_BOARD")):
            raise ValueError(f"changed {NAME} requires a new replay/bias-learning session")
    LAND = land_flags(ULOG, TIMES)
    CLIPPING = IMU.get("accelerometer_clipping")
    ROWS = []
    for OFFSET, TIME in enumerate(TIMES):
        MASK = -1
        if CLIPPING is not None:
            VALUE = float(CLIPPING[START_INDEX + OFFSET])
            if not math.isfinite(VALUE) or not VALUE.is_integer() or not 0 <= VALUE <= 7:
                raise ValueError("invalid accelerometer clipping mask")
            MASK = int(VALUE)
        ROWS.append([TIME, LAND[OFFSET][1], MASK])
    return ROWS


def bias_parameters(ULOG, END):
    """Translate current or legacy logged bias controls into one fixed session."""
    NAMES = ("EKF2_ABL_ACCLIM", "EKF2_ABL_GYRLIM", "EKF2_ABL_TAU")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    if not finite(VALUES) or any(VALUE <= 0 for VALUE in VALUES):
        raise ValueError("invalid bias magnitude/decay parameters")
    SOURCE = "EKF2_IMU_CTRL" if "EKF2_IMU_CTRL" in ULOG.initial_parameters else "EKF2_AID_MASK"
    MASK = float(ULOG.initial_parameters[SOURCE])
    if not math.isfinite(MASK) or not MASK.is_integer() or not 0 <= MASK <= 2**31 - 1:
        raise ValueError(f"invalid {SOURCE} bitmask")
    MASK = int(MASK)
    if SOURCE == "EKF2_IMU_CTRL":
        ACCEL, GYRO = bool(MASK & 2), bool(MASK & 1)
    else:
        # Legacy bit 2 inhibits accelerometer bias; no gyro-bias disable bit.
        ACCEL, GYRO = not bool(MASK & 4), True
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, "EKF2_IMU_CTRL", "EKF2_AID_MASK"):
            raise ValueError(f"changed {NAME} requires a new bias-learning session")
    return [*VALUES, int(ACCEL), int(GYRO)]


def bias_limits(ULOG, END):
    """Use logged per-axis bounds, with explicit EKF2 module-default fallbacks."""
    DEFAULTS = {"EKF2_ABL_LIM": 0.4, "EKF2_GYR_B_LIM": 0.15}
    VALUES = [float(ULOG.initial_parameters.get(NAME, DEFAULT)) for NAME, DEFAULT in DEFAULTS.items()]
    if not finite(VALUES) or any(VALUE < 0 for VALUE in VALUES):
        raise ValueError("invalid bias limits")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in DEFAULTS:
            raise ValueError(f"changed {NAME} requires a new bias-limit session")
    SOURCES = {NAME: "logged" if NAME in ULOG.initial_parameters else "fallback EKF2 module default"
               for NAME in DEFAULTS}
    return VALUES, SOURCES


def gnss_displacement(PREVIOUS, CURRENT):
    """Azimuthal equidistant N/E about the preceding fix; MSL altitude supplies Down.

    Match the reference's spherical projection (radius 6371000 m), independently
    of the replay trajectory's fixed-origin ECEF projection.
    """
    LAT0, LON0, ALT0 = PREVIOUS
    LAT, LON, ALT = CURRENT
    NORTH = math.cos(LAT0) * math.sin(LAT) - math.sin(LAT0) * math.cos(LAT) * math.cos(LON - LON0)
    EAST = math.cos(LAT) * math.sin(LON - LON0)
    COS_DISTANCE = math.sin(LAT0) * math.sin(LAT) + math.cos(LAT0) * math.cos(LAT) * math.cos(LON - LON0)
    DISTANCE = math.acos(max(-1.0, min(1.0, COS_DISTANCE)))
    if DISTANCE > math.pi - 1e-6:
        raise ValueError("GNSS displacement is outside the supported local projection")
    SCALE = 6371000.0 * (DISTANCE / math.sin(DISTANCE) if DISTANCE > 0 else 1.0)
    return [SCALE * NORTH, SCALE * EAST, ALT0 - ALT]


def gnss_quality_rows(ULOG, GPS, TIMES, START, END):
    """Keep bad fixes; they must reach the health monitor instead of disappearing."""
    if "device_id" in GPS and len({int(V) for T, V in zip(TIMES, GPS["device_id"]) if T <= END}) != 1:
        raise ValueError("GNSS receiver changes require a new quality/reference session")
    LAND = land_flags(ULOG, TIMES)
    PREVIOUS, REFERENCE_TIME = None, 0
    ROWS = []
    for INDEX, TIME in enumerate(TIMES):
        if not START < TIME <= END:
            continue
        LAT, LON, _, VELOCITY, _ = gps_reading(GPS, INDEX, REQUIRE_VALID=False)
        ALT_MSL = float(GPS["alt"][INDEX]) * 1e-3
        if not math.isfinite(ALT_MSL):
            raise ValueError("invalid GNSS MSL altitude")
        FIX, SATELLITES, VELOCITY_VALID = (float(GPS[NAME][INDEX])
                                          for NAME in ("fix_type", "satellites_used", "vel_ned_valid"))
        if (not all(V.is_integer() and 0 <= V <= 255 for V in (FIX, SATELLITES))
                or VELOCITY_VALID not in (0, 1)):
            raise ValueError("invalid GNSS fix/satellite/velocity-valid metadata")
        DOPS = [float(GPS[NAME][INDEX]) for NAME in ("hdop", "vdop")]
        # Preserve malformed DOP as invalid input rather than hiding its sign with hypot.
        PDOP = math.hypot(*DOPS) if finite(DOPS) and min(DOPS) >= 0 else math.nan
        SPOOFED = -1
        if "spoofing_state" in GPS:
            VALUE = float(GPS["spoofing_state"][INDEX])
            if VALUE not in (0, 1, 2, 3):
                raise ValueError("invalid GNSS spoofing state")
            SPOOFED = int(VALUE == 3)  # MULTIPLE, matching EKF2; unknown(0) is not a spoof flag.
        CURRENT = (LAT, LON, ALT_MSL)
        DELTA = gnss_displacement(PREVIOUS, CURRENT) if PREVIOUS is not None else [0.0] * 3
        ROWS.append([TIME, FIX, SATELLITES, PDOP, SPOOFED, VELOCITY_VALID, *LAND[INDEX],
                     *VELOCITY, *DELTA, REFERENCE_TIME])
        # EKF2 does not submit messages with invalid NED velocity to its GNSS checks.
        if VELOCITY_VALID:
            PREVIOUS, REFERENCE_TIME = CURRENT, TIME
    return ROWS


def prepare(ULOG, OUTPUT):
    IMU = topic(ULOG, "sensor_combined")
    GPS = topic(ULOG, "vehicle_gps_position")
    MAG = topic(ULOG, "vehicle_magnetometer")
    IMU_TIMES, GPS_TIMES, MAG_TIMES = timestamps(IMU), timestamps(GPS), timestamps(MAG)
    # Keep the seed causal: only the first 0.2 s is used and replay starts at
    # the end of that window. Do not use later flight data to initialize.
    SEED = [INDEX for INDEX, TIME in enumerate(IMU_TIMES) if TIME <= IMU_TIMES[0] + 200000]
    if len(SEED) < 10:
        raise ValueError("insufficient IMU samples for the 0.2 s alignment window")
    START = IMU_TIMES[SEED[-1]]
    MAG_SEED = [INDEX for INDEX, TIME in enumerate(MAG_TIMES) if IMU_TIMES[0] <= TIME <= START]
    if not MAG_SEED:
        raise ValueError("no magnetometer samples in alignment window")
    ACCEL = [sum(xyz(IMU, "accelerometer_m_s2", INDEX)[AXIS] for INDEX in SEED) / len(SEED)
             for AXIS in range(3)]
    FIELD = [sum(xyz(MAG, "magnetometer_ga", INDEX)[AXIS] for INDEX in MAG_SEED) / len(MAG_SEED)
             for AXIS in range(3)]
    # The log's configured declination is a starting assumption, not a truth
    # measurement. Biases remain unknown and are initialized to zero in C++.
    DECLINATION = float(ULOG.initial_parameters["EKF2_MAG_DECL"])
    Q = initial_quaternion(ACCEL, FIELD, math.radians(DECLINATION))
    MAG_REFERENCE = magnetic_reference(Q, FIELD, ULOG)
    MAG_ROWS = magnetic_rows(ULOG, MAG, MAG_TIMES, START, IMU_TIMES[-1])
    BIAS_PARAMETERS = bias_parameters(ULOG, IMU_TIMES[-1])
    BIAS_LIMITS, BIAS_LIMIT_SOURCES = bias_limits(ULOG, IMU_TIMES[-1])
    GNSS_PARAMETERS = gnss_parameters(ULOG, IMU_TIMES[-1])
    QUALITY_PARAMETERS = gnss_quality_parameters(ULOG, IMU_TIMES[-1])
    QUALITY_ROWS = gnss_quality_rows(ULOG, GPS, GPS_TIMES, START, IMU_TIMES[-1])
    VALID = [(INDEX, gps_reading(GPS, INDEX)) for INDEX in range(len(GPS_TIMES))]
    VALID = [(INDEX, READING) for INDEX, READING in VALID if READING is not None]
    GPS_SEED = [(INDEX, READING) for INDEX, READING in VALID if GPS_TIMES[INDEX] <= START]
    if not GPS_SEED or START - GPS_TIMES[GPS_SEED[-1][0]] > 1000000:
        raise ValueError("no valid GNSS seed within one second before initialization")
    ORIGIN_INDEX, ORIGIN = GPS_SEED[-1]
    LAT0, LON0, ALT0, V0, ACCURACY0 = ORIGIN
    ORIGIN_ECEF, BASIS = ecef(LAT0, LON0, ALT0), ned_basis(LAT0, LON0)
    IMU_ROWS, GPS_ROWS = [], []
    for INDEX in range(SEED[-1], len(IMU_TIMES)):
        PERIOD = int(IMU["gyro_integral_dt"][INDEX])
        if (PERIOD <= 0 or PERIOD != int(IMU["accelerometer_integral_dt"][INDEX])
                or int(IMU["accelerometer_timestamp_relative"][INDEX]) != 0):
            raise ValueError("this replay requires synchronized gyro/accel integration intervals")
        VALUES = xyz(IMU, "accelerometer_m_s2", INDEX) + xyz(IMU, "gyro_rad", INDEX)
        if not finite(VALUES):
            raise ValueError("non-finite IMU reading")
        IMU_ROWS.append([IMU_TIMES[INDEX], PERIOD, *VALUES])
    for INDEX in range(len(GPS_TIMES)):
        if not START < GPS_TIMES[INDEX] <= IMU_TIMES[-1]:
            continue
        LAT, LON, ALT, VELOCITY, ACCURACY = gps_reading(GPS, INDEX, REQUIRE_VALID=False)
        DELTA = [X - Y for X, Y in zip(ecef(LAT, LON, ALT), ORIGIN_ECEF)]
        POSITION = [dot(ROW, DELTA) for ROW in BASIS]
        # Rotate receiver-local NED velocity into the fixed origin's NED axes.
        LOCAL_BASIS = ned_basis(LAT, LON)
        VELOCITY_ECEF = [sum(LOCAL_BASIS[AXIS][COLUMN] * VELOCITY[AXIS] for AXIS in range(3))
                         for COLUMN in range(3)]
        GPS_ROWS.append([GPS_TIMES[INDEX], *POSITION, *[dot(ROW, VELOCITY_ECEF) for ROW in BASIS], *ACCURACY])
    if len(IMU_ROWS) < 2 or not GPS_ROWS:
        raise ValueError("no IMU/GNSS replay interval after initialization")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    write_csv(OUTPUT / "initial.csv", INITIAL_HEADER, [[START, *Q, *V0, *ACCURACY0]])
    write_csv(OUTPUT / "imu.csv", IMU_HEADER, IMU_ROWS)
    FLAGS = imu_flags(ULOG, IMU, SEED[-1])
    write_csv(OUTPUT / "imu_flags.csv", "timestamp_us,at_rest,accelerometer_clipping", FLAGS)
    write_csv(OUTPUT / "bias_parameters.csv",
              "maximum_specific_force,maximum_angular_rate,decay_time_constant,accelerometer_enabled,gyroscope_enabled",
              [BIAS_PARAMETERS])
    write_csv(OUTPUT / "bias_limits.csv", "accelerometer,gyroscope", [BIAS_LIMITS])
    write_csv(OUTPUT / "navigation.csv", NAVIGATION_HEADER, GPS_ROWS)
    write_csv(OUTPUT / "gnss_parameters.csv",
              "position_gate,velocity_gate,position_noise,velocity_noise,position_noise_limit", [GNSS_PARAMETERS])
    write_csv(OUTPUT / "gnss_quality_parameters.csv",
              "check_mask,minimum_satellites,maximum_pdop,maximum_horizontal_accuracy,maximum_vertical_accuracy,"
              "maximum_speed_accuracy,maximum_horizontal_drift,maximum_vertical_drift,minimum_health_time_us",
              [QUALITY_PARAMETERS])
    write_csv(OUTPUT / "gnss_quality.csv",
              "timestamp_us,fix_type,satellites,pdop,spoofed,vel_ned_valid,in_air,at_rest,"
              "vn,ve,vd,dn,de,dd,reference_timestamp_us", QUALITY_ROWS)
    write_csv(OUTPUT / "magnetic_reference.csv", "mn,me,md,noise_std,gate_sigma", [MAG_REFERENCE])
    write_csv(OUTPUT / "magnetometer.csv", "timestamp_us,mx,my,mz", MAG_ROWS)
    return {"start_timestamp_us": START, "end_timestamp_us": IMU_TIMES[-1],
            "imu_rows": len(IMU_ROWS), "gnss_rows": len(GPS_ROWS),
            "magnetometer_rows": len(MAG_ROWS),
            "gnss_invalid_rows": len(GPS_TIMES) - len(VALID),
            "gnss_seed_timestamp_us": GPS_TIMES[ORIGIN_INDEX],
            "gnss_valid_rows_outside_replay": sum(not START < GPS_TIMES[I] <= IMU_TIMES[-1] for I, _ in VALID),
            "origin_lat_deg": math.degrees(LAT0), "origin_lon_deg": math.degrees(LON0),
            "origin_alt_ellipsoid_m": ALT0, "initialization": "first 0.2 s accel/mag; latest valid GNSS",
            "magnetic_declination_deg": DECLINATION, "gnss_delay_compensation_ms": 0,
            "gnss_parameters": dict(zip(("position_gate", "velocity_gate", "position_noise_std_m",
                                          "velocity_noise_std_mps", "position_noise_limit_m"), GNSS_PARAMETERS)),
            "gnss_quality_parameters": dict(zip(("check_mask", "minimum_satellites", "maximum_pdop",
                "maximum_horizontal_accuracy", "maximum_vertical_accuracy", "maximum_speed_accuracy",
                "maximum_horizontal_drift", "maximum_vertical_drift", "minimum_health_time_us"), QUALITY_PARAMETERS)),
            "gnss_health_time_source": "EKF2_REQ_GPS_H" if "EKF2_REQ_GPS_H" in ULOG.initial_parameters
                else "explicit 10 s reference default; legacy log has no EKF2_REQ_GPS_H",
            "gnss_quality_motion": "causal landed/at_rest; unknown required flags block quality and reset qualification",
            "gnss_quality_spoofing": "missing field is unavailable; MULTIPLE(3) sets spoofed as in EKF2",
            "gnss_quality_projection": "adjacent-fix spherical N/E, radius 6371000 m; MSL Down; receiver-local velocity",
            "gnss_fusion": "3D velocity, horizontal position, height; each gates/corrects its latest prior",
            "gnss_noise": "logged floors; horizontal std capped by NOAID_NOISE; vertical velocity std scaled by 1.5",
            "gnss_height_reference": "sole GNSS height reference; no height bias or additional bias variance",
            "gnss_accuracy_frame": "reported accuracy axes treated as fixed local NED; local-flight approximation",
            "gnss_lifecycle": "GNSS-only: horizontal data/quality timeout 7 s, height data timeout 1 s; horizontal reset at both aids >7 s or position >14 s, height fusion timeout 5 s",
            "gnss_startup": "seeded attitude/frame is assumed aligned; each group resets on accepted startup/re-entry; execution order: 3D velocity, horizontal position, height",
            "gnss_resets": "horizontal timeout resets replace correction; stale height triggers Down-only velocity reset after normal velocity correction and before height correction; height reset follows rejected overdue correction; separate deltas/counts; numerical failures abort",
            "gnss_limits": "quality precedes lifecycle and gates; single replay clock; co-located IMU/GNSS; no alternate aiding, height-bias estimation, bad-vertical-acceleration override or emergency yaw",
            "magnetic_reference_n_ut": MAG_REFERENCE[:3],
            "magnetic_noise_std_ut": MAG_REFERENCE[3], "magnetic_gate_sigma": MAG_REFERENCE[4],
            "magnetic_reference_source": "R(q_seed) times first 0.2 s mean field; not independent WMM/truth",
            "magnetic_measurement": "vehicle_magnetometer instance 0, calibrated body FRD, Gauss times 100 -> microtesla",
            "magnetic_timing": "newest topic sample at each IMU tick; superseded samples counted; no sensor-delay compensation",
            "magnetic_lifecycle": "tilt alignment, startup health and sample count; retry after ordinary stops; timeout heading reset on fresh healthy data without horizontal aid or at rest; numerical failures abort",
            "magnetic_reset": "fixed reference retained; tilt-preserving yaw and covariance reset; separate applied quaternion deltas/counts; explicit replay heading noise, no magnetic-state reset",
            "magnetic_identity": "fixed CSV session identity 1, generation 0; not hardware device_id/calibration_count",
            "magnetic_calibration": "use published calibration; no additional CAL_MAG or EKF2_MAGBIAS subtraction",
            "magnetic_source_limits": "reject recorded source/calibration changes; missing metadata cannot establish absence of changes",
            "at_rest_unavailable_rows": sum(ROW[1] == -1 for ROW in FLAGS),
            "clipping_unavailable_rows": sum(ROW[2] == -1 for ROW in FLAGS),
            "bias_parameters": dict(zip(("maximum_specific_force", "maximum_angular_rate", "decay_time_constant",
                                          "accelerometer_enabled", "gyroscope_enabled"), BIAS_PARAMETERS)),
            "bias_control_source": "EKF2_IMU_CTRL" if "EKF2_IMU_CTRL" in ULOG.initial_parameters else "EKF2_AID_MASK",
            "bias_limits": dict(zip(("accelerometer_m_s2", "gyroscope_rad_s"), BIAS_LIMITS)),
            "bias_limit_sources": BIAS_LIMIT_SOURCES,
            "bias_limiting": "per body axis after each successful correction group; covariance unchanged by limiting; not PX4 scalar-sequential saturation",
            "bias_learning": "pre-prediction bias-corrected decaying peaks; all corrections share six permissions; no recovery timer or gyro clipping condition",
            "bias_clipping_compatibility": "missing accel clipping assumed clear ONLY for bias learning; -1 retained for gravity fusion; not evidence of no clipping",
            "bias_mode_limits": "no fake position/height modes; vertical acceleration health requires GNSS lifecycle and known clipping; its fault inhibits accel bias and selects prediction noise",
            "imu_source_limits": "reject recorded IMU selection/calibration changes; missing metadata does not prove continuity or sensor health",
            "timing": "GNSS message timestamp, fused on following IMU tick; no physical latency compensation",
            "height": "ellipsoidal GNSS altitude projected to fixed local NED; no barometer/rangefinder",
            "imu_gap_policy": "hold previous IMU rate over missing interval; report every gap",
            "claim": "wiring/numerical smoke test, not accuracy or statistical consistency validation"}


def run_replay(LOG_FILE, BINARY, COMPARE_MAG=False):
    BINARY = BINARY.resolve()
    if not BINARY.is_file() or not os.access(BINARY, os.X_OK):
        raise ValueError(f"replay executable not found or not executable: {BINARY}; build it with CMake first")
    if not LOG_FILE.is_file():
        raise ValueError(f"ULog file not found: {LOG_FILE}")
    if LOG_FILE.stem in ("", ".", ".."):
        raise ValueError("ULog filename must have a nonempty name before the extension")
    OUTPUT = Path("output") / LOG_FILE.stem
    LOG_FILE = LOG_FILE.resolve()
    if OUTPUT.is_symlink() or OUTPUT.parent.is_symlink():
        raise ValueError(f"replay output path must not be a symlink: {OUTPUT}")
    if any(PATH.is_relative_to(OUTPUT.resolve()) for PATH in (LOG_FILE, BINARY)):
        raise ValueError("ULog and replay executable must be outside the replay output directory")
    from pyulog import ULog
    from tqdm import tqdm as TQDM
    with TQDM(total=3 + int(COMPARE_MAG), desc="Read ULog", dynamic_ncols=True, bar_format=PROGRESS_FORMAT) as PROGRESS:
        ULOG = ULog(str(LOG_FILE), message_name_filter_list=[
            "sensor_combined", "vehicle_gps_position", "vehicle_magnetometer", "vehicle_land_detected", "sensor_selection"])
        PROGRESS.update(1)
        PROGRESS.set_description_str("Prepare CSV / metadata")
        if OUTPUT.exists():
            shutil.rmtree(OUTPUT)
            PROGRESS.write(f"Removed previous replay output: {OUTPUT}")
        OUTPUT.mkdir(parents=True, exist_ok=False)
        META = prepare(ULOG, OUTPUT / "input")
        with LOG_FILE.open("rb") as STREAM:
            HASH = hashlib.sha256()
            for CHUNK in iter(lambda: STREAM.read(1048576), b""):
                HASH.update(CHUNK)
        META.update(source=str(LOG_FILE.resolve()), sha256=HASH.hexdigest(),
                    px4_version=ULOG.msg_info_dict.get("ver_sw"), hardware=ULOG.msg_info_dict.get("ver_hw"),
                    logged_dropout_count=len(ULOG.dropouts))
        with (OUTPUT / "input/source.json").open("x") as STREAM:
            json.dump(META, STREAM, indent=2, allow_nan=False)
            STREAM.write("\n")
        PROGRESS.write(f"Prepared {META['imu_rows']} IMU rows, {META['gnss_rows']} GNSS events, "
                       f"{META['magnetometer_rows']} magnetometer events")
        PROGRESS.update(1)
        PROGRESS.set_description_str("Run C++ replay")
        RESULT = subprocess.run([str(BINARY), str(OUTPUT / "input"), str(OUTPUT / "result")],
                                check=True, stdout=subprocess.PIPE, text=True)
        PROGRESS.update(1)
        if COMPARE_MAG:
            PROGRESS.set_description_str("Run without mag correction")
            BASELINE = subprocess.run([str(BINARY), str(OUTPUT / "input"), str(OUTPUT / "result_no_mag"), "--no-mag"],
                                      check=True, stdout=subprocess.PIPE, text=True)
            PROGRESS.update(1)
        PROGRESS.set_description_str("Replay complete")
    print(RESULT.stdout, end="", flush=True)
    if COMPARE_MAG:
        print("Without mag correction (same inputs and initialization):", flush=True)
        print(BASELINE.stdout, end="", flush=True)
    print(f"Output: {OUTPUT}", flush=True)
    return OUTPUT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__,
                                     epilog="Results replace output/<ulog filename without .ulg>/ on each run.")
    PARSER.add_argument("ulog", type=Path)
    PARSER.add_argument("--binary", type=Path, default=Path("build/demo/replay_ins"),
                        help="prebuilt C++ executable (default: build/demo/replay_ins)")
    PARSER.add_argument("--plot", action="store_true", help="save PNG plots and display them with matplotlib")
    PARSER.add_argument("--compare-mag", action="store_true",
                        help="also replay identical inputs without mag correction; --plot adds the comparison")
    ARGS = PARSER.parse_args()
    try:
        if ARGS.plot:
            # Check the optional dependency before creating the output directory.
            import matplotlib.pyplot
        OUTPUT = run_replay(ARGS.ulog, ARGS.binary, ARGS.compare_mag)
        if ARGS.plot:
            plot_replay(OUTPUT)
    except subprocess.CalledProcessError as ERROR:
        PARSER.exit(1, f"error: C++ replay failed (exit {ERROR.returncode})\n")
    except ImportError as ERROR:
        PARSER.exit(2, f"error: {ERROR}; install dependencies with: "
                      "python3 -m pip install -r requirements.txt\n")
    except (OSError, KeyError, ValueError) as ERROR:
        PARSER.exit(2, f"error: {ERROR}\n")


if __name__ == "__main__":
    main()
