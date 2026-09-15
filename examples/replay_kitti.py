#!/usr/bin/env python3
"""Replay KITTI Raw unsynced OXTS through the shared C++ INS example.

Only oxts/data/*.txt and oxts/timestamps.txt are needed. OXTS supplies both
inertial samples and navigation estimates, so this is not an independent
GNSS/IMU accuracy benchmark. No images, LiDAR, or camera calibration are used.
"""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess

from replay_common import (IMU_HEADER, INITIAL_HEADER, NAVIGATION_HEADER, PROGRESS_FORMAT,
                           dot, ecef, ned_basis, plot_replay, write_csv)


OXTS_FIELDS = (
    "lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz wf wl wu "
    "pos_accuracy vel_accuracy navstat numsats posmode velmode orimode"
).split()
# Demo observation standard deviations: horizontal position, vertical position,
# and velocity. OXTS does not supply the full independent observation covariance.
# These match the shared C++ replay floors, not a calibrated OXTS noise model.
NAVIGATION_STD = (1.0, 1.5, 0.3)
# Explicit demo gap policy, not reconstructed measurements or calibrated noise.
GAP_THRESHOLD_US = 50000
MAX_GAP_US = 2000000
HOLD_STEP_US = 10000


def timestamp_ns(TEXT):
    """Parse KITTI's local date/time without float epoch rounding or timezone conversion."""
    SECONDS, SEPARATOR, FRACTION = TEXT.strip().partition(".")
    if SEPARATOR != "." or not FRACTION.isascii() or not FRACTION.isdigit() or len(FRACTION) > 9:
        raise ValueError(f"invalid KITTI timestamp: {TEXT!r}")
    DATE = datetime.strptime(SECONDS, "%Y-%m-%d %H:%M:%S")
    WHOLE_SECONDS = DATE.toordinal() * 86400 + DATE.hour * 3600 + DATE.minute * 60 + DATE.second
    return WHOLE_SECONDS * 1000000000 + int(FRACTION.ljust(9, "0"))


def read_oxts(DRIVE):
    from tqdm import tqdm as TQDM

    OXTS = DRIVE / "oxts"
    STAMPS = (OXTS / "timestamps.txt").read_text().splitlines()
    FILES = sorted((OXTS / "data").glob("*.txt"))
    if len(FILES) < 2 or len(FILES) != len(STAMPS):
        raise ValueError("expected matching OXTS timestamps and at least two numbered data files")
    HASH = hashlib.sha256()
    RECORDS = []
    for INDEX, (FILE, STAMP) in enumerate(TQDM(zip(FILES, STAMPS), total=len(FILES),
                                            desc="Read OXTS", dynamic_ncols=True)):
        if FILE.name != f"{INDEX:010d}.txt":
            raise ValueError(f"missing or misnumbered OXTS frame before {FILE.name}")
        TIME = timestamp_ns(STAMP)
        TEXT = FILE.read_text()
        VALUES = [float(VALUE) for VALUE in TEXT.split()]
        if len(VALUES) != len(OXTS_FIELDS) or not all(math.isfinite(VALUE) for VALUE in VALUES):
            raise ValueError(f"{FILE.name}: expected 30 finite OXTS fields")
        PACKET = dict(zip(OXTS_FIELDS, VALUES))
        if abs(PACKET["lat"]) > 90 or abs(PACKET["lon"]) > 180:
            raise ValueError(f"{FILE.name}: latitude/longitude out of range")
        if (abs(PACKET["roll"]) > math.pi or abs(PACKET["pitch"]) > math.pi / 2
                or abs(PACKET["yaw"]) > math.pi):
            raise ValueError(f"{FILE.name}: roll/pitch/yaw out of range")
        MODES = [PACKET[KEY] for KEY in ("posmode", "velmode", "orimode")]
        if any(MODE != int(MODE) or not 0 <= MODE <= 30 for MODE in MODES):
            raise ValueError(f"{FILE.name}: invalid OXTS navigation modes; interpolated (-1) "
                             "outage records are not accepted as measured IMU data")
        if PACKET["navstat"] != int(PACKET["navstat"]) or not 0 <= PACKET["navstat"] <= 23:
            raise ValueError(f"{FILE.name}: invalid navigation status")
        HASH.update(FILE.name.encode() + b"\n" + STAMP.encode() + b"\n" + TEXT.encode() + b"\n")
        RECORDS.append((TIME, PACKET))
    RECORDS, TIMING = order_records(RECORDS)
    return RECORDS, HASH.hexdigest(), TIMING


def order_records(RECORDS):
    """Offline ordering keeps each packet paired with its original timestamp."""
    REVERSALS = sum(B[0] < A[0] for A, B in zip(RECORDS, RECORDS[1:]))
    ORDERED = sorted(RECORDS, key=lambda RECORD: RECORD[0])
    ORIGIN_NS = ORDERED[0][0]
    UNIQUE = []
    DUPLICATES = 0
    for TIME_NS, PACKET in ORDERED:
        TIME = 1000000 + (TIME_NS - ORIGIN_NS) // 1000
        if UNIQUE and TIME == UNIQUE[-1][0]:
            if TIME_NS != PREVIOUS_NS or PACKET != UNIQUE[-1][1]:
                raise ValueError("conflicting OXTS packets or timestamp collision at microsecond resolution")
            DUPLICATES += 1
            continue
        UNIQUE.append((TIME, PACKET))
        PREVIOUS_NS = TIME_NS
    INTERVALS = [B[0] - A[0] for A, B in zip(UNIQUE, UNIQUE[1:])]
    if not INTERVALS or not 5000 <= statistics.median(INTERVALS) <= 20000:
        raise ValueError("use Raw unsynced/extract OXTS (~100 Hz), not synced (~10 Hz)")
    if max(INTERVALS) > MAX_GAP_US:
        raise ValueError("OXTS gap exceeds the demo's 2 s hold limit")
    return UNIQUE, {"source_rows": len(RECORDS), "timestamp_reversals": REVERSALS,
                    "identical_duplicates_removed": DUPLICATES}


def navigation_valid(PACKET):
    # navstat=4 is a locked INS solution. GPS modes describe receiver aiding,
    # not validity of the fused attitude; orimode=0 does NOT invalidate q_nb.
    # Do not aid from position/velocity modes None, Search or unknown solutions.
    # Source: OxfordTechnicalSolutions/NCOMdecoder, NavigationStatus/GpsXModeName.
    return PACKET["navstat"] == 4 and 3 <= PACKET["posmode"] <= 9 and 2 <= PACKET["velmode"] <= 9


def initial_quaternion(PACKET):
    """KITTI FLU -> ENU attitude converted to scalar-first FRD -> NED q_nb.

    R_nb = C_enu_to_ned * Rz(yaw) * Ry(pitch) * Rx(roll) * C_frd_to_flu.
    Equivalently, the NED/FRD Euler angles are (roll, -pitch, pi/2 - yaw).
    KITTI's yaw is zero toward East and increases toward North.
    """
    ROLL, PITCH, YAW = PACKET["roll"], -PACKET["pitch"], math.pi / 2 - PACKET["yaw"]
    CR, SR = math.cos(ROLL / 2), math.sin(ROLL / 2)
    CP, SP = math.cos(PITCH / 2), math.sin(PITCH / 2)
    CY, SY = math.cos(YAW / 2), math.sin(YAW / 2)
    return [CR * CP * CY + SR * SP * SY, SR * CP * CY - CR * SP * SY,
            CR * SP * CY + SR * CP * SY, CR * CP * SY - SR * SP * CY]


def inertial_sample(PACKET):
    # Use body xyz, NOT the level forward/left/up (af/al/au, wf/wl/wu) fields.
    # FLU -> FRD is a proper rotation: flip Y and Z for both polar/axial vectors.
    # ax/ay/az are accelerometer specific force (level rest: az ~= +g).
    # Do not subtract gravity here: the core adds gravity in navigation axes.
    return [PACKET["ax"], -PACKET["ay"], -PACKET["az"],
            PACKET["wx"], -PACKET["wy"], -PACKET["wz"]]


def navigation_sample(PACKET, ORIGIN, BASIS):
    LAT, LON = math.radians(PACKET["lat"]), math.radians(PACKET["lon"])
    POSITION_ECEF = ecef(LAT, LON, PACKET["alt"])
    POSITION = [dot(ROW, [A - B for A, B in zip(POSITION_ECEF, ORIGIN)]) for ROW in BASIS]
    # OXTS vn/ve/vu refer to the tangent frame at the current location.
    # Rotate velocity through ECEF so all rows use the same initial NED basis.
    LOCAL_BASIS = ned_basis(LAT, LON)
    LOCAL_VELOCITY = [PACKET["vn"], PACKET["ve"], -PACKET["vu"]]
    VELOCITY_ECEF = [dot(COLUMN, LOCAL_VELOCITY) for COLUMN in zip(*LOCAL_BASIS)]
    VELOCITY = [dot(ROW, VELOCITY_ECEF) for ROW in BASIS]
    return [*POSITION, *VELOCITY]


def prepare(RECORDS, NAVIGATION_HZ):
    SEED_INDEX = next((INDEX for INDEX, (_, PACKET) in enumerate(RECORDS) if navigation_valid(PACKET)), None)
    if SEED_INDEX is None:
        raise ValueError("no locked OXTS solution with position and velocity aiding for initialization")
    RECORDS = RECORDS[SEED_INDEX:]
    START, SEED = RECORDS[0]
    LAT0, LON0 = math.radians(SEED["lat"]), math.radians(SEED["lon"])
    ORIGIN = ecef(LAT0, LON0, SEED["alt"])
    BASIS = ned_basis(LAT0, LON0)
    INITIAL = [START, *initial_quaternion(SEED), *navigation_sample(SEED, ORIGIN, BASIS)[3:], *NAVIGATION_STD]
    # The first row initializes the runner; its interval is not integrated.
    IMU_ROWS = [[START, 1, *inertial_sample(SEED)]]
    NAVIGATION_ROWS = []
    GAPS = []
    SKIPPED_UPDATES = 0
    PERIOD = round(1000000 / NAVIGATION_HZ)
    NEXT_UPDATE = START + PERIOD
    for (PREVIOUS_TIME, PREVIOUS), (TIME, PACKET) in zip(RECORDS, RECORDS[1:]):
        # OXTS gives instantaneous samples, not PX4-style interval averages.
        # Explicit left-endpoint hold over each observed timestamp interval.
        DELTA = TIME - PREVIOUS_TIME
        STEPS = (DELTA + HOLD_STEP_US - 1) // HOLD_STEP_US if DELTA > GAP_THRESHOLD_US else 1
        if STEPS > 1:
            GAPS.append({"start_timestamp_us": PREVIOUS_TIME, "end_timestamp_us": TIME,
                         "held_interval_us": DELTA, "synthetic_rows": STEPS - 1})
        SAMPLE = inertial_sample(PREVIOUS)
        TICK = PREVIOUS_TIME
        for STEP in range(1, STEPS + 1):
            NEXT_TICK = PREVIOUS_TIME + DELTA * STEP // STEPS
            IMU_ROWS.append([NEXT_TICK, NEXT_TICK - TICK, *SAMPLE])
            TICK = NEXT_TICK
        if TIME >= NEXT_UPDATE:
            if navigation_valid(PACKET):
                NAVIGATION_ROWS.append([TIME, *navigation_sample(PACKET, ORIGIN, BASIS), *NAVIGATION_STD])
            else:
                SKIPPED_UPDATES += 1
            NEXT_UPDATE += ((TIME - NEXT_UPDATE) // PERIOD + 1) * PERIOD
    if not NAVIGATION_ROWS:
        raise ValueError("drive is too short for a navigation correction at the requested rate")
    META = {
        "dataset": "KITTI Raw unsynced OXTS", "imu_rows": len(IMU_ROWS),
        "navigation_rows": len(NAVIGATION_ROWS), "navigation_hz": NAVIGATION_HZ,
        "initial_rows_skipped": SEED_INDEX, "navigation_events_skipped": SKIPPED_UPDATES,
        "gap_holds": GAPS, "synthetic_imu_rows": sum(GAP["synthetic_rows"] for GAP in GAPS),
        "origin_lat_deg": SEED["lat"], "origin_lon_deg": SEED["lon"], "origin_alt_m": SEED["alt"],
        "frame": "body FRD, fixed initial local NED; geodetic via WGS84 ECEF",
        "height": "OXTS altitude used as WGS84 altitude; no geoid/datum correction",
        "initialization": "first locked OXTS solution with position/velocity aiding; position zero; biases zero",
        "timestamp": "offline timestamp sort; identical duplicates removed; rebased then truncated to microseconds",
        "imu_sampling": "hold previous body specific force/angular rate to the next timestamp",
        "navigation_source": "decimated OXTS fused navigation estimates, not independent raw GNSS",
        "navigation_std_m_m_mps": NAVIGATION_STD,
        "noise": "fixed demo observation stds; shared C++ replay process/initial covariance settings",
        "timing": "navigation fused at its OXTS/IMU tick; no latency compensation",
        "gap_policy": "hold previous IMU over gaps >50 ms, substeps <=10 ms; reject gaps >2 s or interpolated (-1) records",
        "gap_noise": "same demo per-sample noise per held substep, not independent measurements or a calibrated outage covariance",
        "claim": "replay demonstration only; OXTS inputs are not independent ground truth",
    }
    return INITIAL, IMU_ROWS, NAVIGATION_ROWS, META


def run_replay(DRIVE, BINARY, NAVIGATION_HZ):
    from tqdm import tqdm as TQDM

    DRIVE, BINARY = DRIVE.resolve(), BINARY.resolve()
    if not BINARY.is_file() or not os.access(BINARY, os.X_OK):
        raise ValueError(f"replay executable not found or not executable: {BINARY}; build it with CMake first")
    if not DRIVE.is_dir() or not (DRIVE / "oxts").is_dir():
        raise ValueError(f"expected a KITTI drive directory containing oxts/: {DRIVE}")
    OUTPUT = Path("output") / ("kitti-" + DRIVE.name)
    if OUTPUT.is_symlink() or OUTPUT.parent.is_symlink():
        raise ValueError(f"replay output path must not be a symlink: {OUTPUT}")
    if (DRIVE.is_relative_to(OUTPUT.resolve()) or OUTPUT.resolve().is_relative_to(DRIVE)
            or (DRIVE / "oxts").resolve().is_relative_to(OUTPUT.resolve())
            or BINARY.is_relative_to(OUTPUT.resolve())):
        raise ValueError("dataset, executable and replay output paths must not overlap")
    RECORDS, HASH, TIMING = read_oxts(DRIVE)
    INITIAL, IMU_ROWS, NAVIGATION_ROWS, META = prepare(RECORDS, NAVIGATION_HZ)
    # Validate the entire selected drive before replacing a previous result.
    with TQDM(total=2, desc="Prepare CSV / metadata", dynamic_ncols=True, bar_format=PROGRESS_FORMAT) as PROGRESS:
        if OUTPUT.exists():
            if not OUTPUT.is_dir():
                raise ValueError(f"replay output already exists and is not a directory: {OUTPUT}")
            shutil.rmtree(OUTPUT)
            PROGRESS.write(f"Removed previous replay output: {OUTPUT}")
        INPUT = OUTPUT / "input"
        INPUT.mkdir(parents=True, exist_ok=False)
        write_csv(INPUT / "initial.csv", INITIAL_HEADER, [INITIAL])
        write_csv(INPUT / "imu.csv", IMU_HEADER, IMU_ROWS)
        write_csv(INPUT / "navigation.csv", NAVIGATION_HEADER, NAVIGATION_ROWS)
        META.update(source=str(DRIVE), oxts_sha256=HASH, **TIMING)
        with (INPUT / "source.json").open("x") as STREAM:
            json.dump(META, STREAM, indent=2, allow_nan=False)
            STREAM.write("\n")
        PROGRESS.write(f"Prepared {len(IMU_ROWS)} IMU rows, {len(NAVIGATION_ROWS)} OXTS navigation events")
        PROGRESS.write(f"Timestamp reversals: {TIMING['timestamp_reversals']}; "
                       f"identical duplicates removed: {TIMING['identical_duplicates_removed']}")
        GAP_SECONDS = sum(GAP["held_interval_us"] for GAP in META["gap_holds"]) * 1e-6
        PROGRESS.write(f"Adapter IMU gap holds: {len(META['gap_holds'])} ({GAP_SECONDS:.3f} s); "
                       f"{META['synthetic_imu_rows']} synthetic rows, not new measurements")
        PROGRESS.write(f"Navigation events skipped (receiver position/velocity unavailable): "
                       f"{META['navigation_events_skipped']}")
        PROGRESS.update(1)
        PROGRESS.set_description_str("Run C++ replay")
        RESULT = subprocess.run([str(BINARY), str(INPUT), str(OUTPUT / "result")],
                                check=True, stdout=subprocess.PIPE, text=True)
        PROGRESS.update(1)
        PROGRESS.set_description_str("Replay complete")
    print(RESULT.stdout, end="", flush=True)
    print(f"Output: {OUTPUT}", flush=True)
    return OUTPUT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__,
                                     epilog="Results replace output/kitti-<drive directory name>/ on each run.")
    PARSER.add_argument("drive", type=Path, help="unsynced/extract drive directory containing oxts/")
    PARSER.add_argument("--binary", type=Path, default=Path("build/demo/replay_ins"),
                        help="prebuilt C++ executable (default: build/demo/replay_ins)")
    PARSER.add_argument("--navigation-hz", type=float, default=10.0,
                        help="OXTS position/velocity aiding rate, 0.1 to 100 Hz (default: 10 Hz)")
    PARSER.add_argument("--plot", action="store_true", help="save PNG plots and display them with matplotlib")
    ARGS = PARSER.parse_args()
    if not math.isfinite(ARGS.navigation_hz) or not 0.1 <= ARGS.navigation_hz <= 100:
        PARSER.error("--navigation-hz must be finite and in [0.1, 100]")
    try:
        if ARGS.plot:
            import matplotlib.pyplot
        OUTPUT = run_replay(ARGS.drive, ARGS.binary, ARGS.navigation_hz)
        if ARGS.plot:
            plot_replay(OUTPUT, SOURCE_LABEL="OXTS navigation")
    except subprocess.CalledProcessError as ERROR:
        PARSER.exit(1, f"error: C++ replay failed (exit {ERROR.returncode})\n")
    except ImportError as ERROR:
        PARSER.exit(2, f"error: {ERROR}; install dependencies with: "
                      "python3 -m pip install -r requirements.txt\n")
    except (OSError, KeyError, ValueError) as ERROR:
        PARSER.exit(2, f"error: {ERROR}\n")


if __name__ == "__main__":
    main()
