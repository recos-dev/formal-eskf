#!/usr/bin/env python3
"""Replay a legacy PX4 ULog through the prebuilt C++ INS example.

Only sensor_combined, vehicle_gps_position and vehicle_magnetometer are consumed.
PX4 estimator outputs are not inputs. Magnetic measurements initialize attitude
only; this first replay does not fuse magnetometer or accelerometer corrections.
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


def gps_reading(GPS, INDEX):
    LAT = math.radians(float(GPS["lat"][INDEX]) * 1e-7)
    LON = math.radians(float(GPS["lon"][INDEX]) * 1e-7)
    ALT = float(GPS["alt_ellipsoid"][INDEX]) * 1e-3
    VELOCITY = [float(GPS[FIELD][INDEX]) for FIELD in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s")]
    ACCURACY = [float(GPS[FIELD][INDEX]) for FIELD in ("eph", "epv", "s_variance_m_s")]
    if (int(GPS["fix_type"][INDEX]) < 3 or not GPS["vel_ned_valid"][INDEX]
            or not finite([LAT, LON, ALT, *VELOCITY, *ACCURACY])
            or abs(LAT) > math.pi / 2 or abs(LON) > math.pi or min(ACCURACY) <= 0):
        return None
    return LAT, LON, ALT, VELOCITY, ACCURACY


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
    for INDEX, (LAT, LON, ALT, VELOCITY, ACCURACY) in VALID:
        if not START < GPS_TIMES[INDEX] <= IMU_TIMES[-1]:
            continue
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
    write_csv(OUTPUT / "navigation.csv", NAVIGATION_HEADER, GPS_ROWS)
    return {"start_timestamp_us": START, "end_timestamp_us": IMU_TIMES[-1],
            "imu_rows": len(IMU_ROWS), "gnss_rows": len(GPS_ROWS),
            "gnss_invalid_rows": len(GPS_TIMES) - len(VALID),
            "gnss_seed_timestamp_us": GPS_TIMES[ORIGIN_INDEX],
            "gnss_valid_rows_outside_replay": len(VALID) - len(GPS_ROWS),
            "origin_lat_deg": math.degrees(LAT0), "origin_lon_deg": math.degrees(LON0),
            "origin_alt_ellipsoid_m": ALT0, "initialization": "first 0.2 s accel/mag; latest valid GNSS",
            "magnetic_declination_deg": DECLINATION, "gnss_delay_compensation_ms": 0,
            "timing": "GNSS message timestamp, fused on following IMU tick; no physical latency compensation",
            "height": "ellipsoidal GNSS altitude projected to fixed local NED; no barometer/rangefinder",
            "imu_gap_policy": "hold previous IMU rate over missing interval; report every gap",
            "claim": "wiring/numerical smoke test, not accuracy or statistical consistency validation"}


def run_replay(LOG_FILE, BINARY):
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
    with TQDM(total=3, desc="Read ULog", dynamic_ncols=True, bar_format=PROGRESS_FORMAT) as PROGRESS:
        ULOG = ULog(str(LOG_FILE), message_name_filter_list=[
            "sensor_combined", "vehicle_gps_position", "vehicle_magnetometer"])
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
        PROGRESS.write(f"Prepared {META['imu_rows']} IMU rows, {META['gnss_rows']} GNSS events")
        PROGRESS.update(1)
        PROGRESS.set_description_str("Run C++ replay")
        RESULT = subprocess.run([str(BINARY), str(OUTPUT / "input"), str(OUTPUT / "result")],
                                check=True, stdout=subprocess.PIPE, text=True)
        PROGRESS.update(1)
        PROGRESS.set_description_str("Replay complete")
    print(RESULT.stdout, end="", flush=True)
    print(f"Output: {OUTPUT}", flush=True)
    return OUTPUT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__,
                                     epilog="Results replace output/<ulog filename without .ulg>/ on each run.")
    PARSER.add_argument("ulog", type=Path)
    PARSER.add_argument("--binary", type=Path, default=Path("build/demo/replay_ins"),
                        help="prebuilt C++ executable (default: build/demo/replay_ins)")
    PARSER.add_argument("--plot", action="store_true", help="save PNG plots and display them with matplotlib")
    ARGS = PARSER.parse_args()
    try:
        if ARGS.plot:
            # Check the optional dependency before creating the output directory.
            import matplotlib.pyplot
        OUTPUT = run_replay(ARGS.ulog, ARGS.binary)
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
