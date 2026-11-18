#!/usr/bin/env python3
"""Measure PX4 projection/frame approximations against independent geodesy."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys

import geographiclib
from geographiclib.geodesic import Geodesic
import numpy as np
import pyproj
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "examples"))
import replay_common

GEOD = Geodesic.WGS84
LATITUDES = [-80., -60., -45., -25., 0., 25., 45., 60., 80., 89.9]
LONGITUDES = [0., 120., 179.999]
DISTANCES = [0., 100., 500., 1000., 2000., 5000., 10000., 20000., 50000., 100000.]


def sha256(PATH):
    return hashlib.sha256(PATH.read_bytes()).hexdigest()


def compile_probe(PX4, OUT):
    BUILD = PX4 / "build/px4_sitl_formal_eskf"
    DB = json.loads((BUILD / "compile_commands.json").read_text())
    ENTRY = next(V for V in DB if Path(V["file"]) == PX4 / "src/lib/geo/geo.cpp")
    FLAGS = [V for V in shlex.split(ENTRY["command"]) if V.startswith(("-I", "-D"))]
    SOURCE = OUT / "projection_probe.cpp"
    SOURCE.write_bytes((REPO / "integrations/px4/tests/ProjectionProbe.cpp").read_bytes())
    COMMAND = ["c++", "-std=c++20", "-O2", "-ffunction-sections", "-fdata-sections",
               *FLAGS, str(SOURCE), str(BUILD / "src/lib/geo/libgeo.a"),
               "-Wl,--gc-sections", "-o", str(OUT / "projection_probe")]
    (OUT / "compile-command.json").write_text(json.dumps(COMMAND, indent=2) + "\n")
    RESULT = subprocess.run(COMMAND, text=True, capture_output=True)
    (OUT / "compile.log").write_text(RESULT.stdout + RESULT.stderr)
    RESULT.check_returncode()
    return OUT / "projection_probe"


def geographic_cases():
    CASES = []
    ORACLES = {}
    for LAT0 in LATITUDES:
        for LON0 in LONGITUDES:
            AEQD = Transformer.from_pipeline(f'+proj=aeqd +R=6371000 +lat_0={LAT0} +lon_0={LON0}')
            TOPO = Transformer.from_pipeline(
                '+proj=pipeline +step +proj=cart +ellps=WGS84 '
                f'+step +proj=topocentric +ellps=WGS84 +lat_0={LAT0} +lon_0={LON0} +h_0=0')
            ORACLES[LAT0, LON0] = AEQD, TOPO
            for DISTANCE in DISTANCES:
                for AZIMUTH in ([0.] if DISTANCE == 0 else range(0, 360, 45)):
                    DEST = GEOD.Direct(LAT0, LON0, AZIMUTH, DISTANCE)
                    CASES.append((LAT0, LON0, DISTANCE, AZIMUTH, DEST['lat2'], DEST['lon2']))
    return CASES, ORACLES


def compare(CASES, ORACLES, VALUES):
    ROWS = []
    REPLAY_MAX = 0.
    HEIGHT_COMPARISONS = 0
    for CASE, ACTUAL in zip(CASES, VALUES):
        LAT0, LON0, DISTANCE, AZIMUTH, LAT, LON = CASE
        AEQD, TOPO = ORACLES[LAT0, LON0]
        EAST, NORTH = AEQD.transform(LON, LAT)
        SPHERE = np.array([NORTH, EAST])
        ACTUAL_NED = ACTUAL[:2]
        ERROR = np.linalg.norm(ACTUAL_NED-SPHERE)
        QUANTIZATION = np.linalg.norm(np.spacing(np.abs(ACTUAL_NED).astype(np.float32)).astype(float))/2
        if ERROR > QUANTIZATION + 2e-6:
            raise RuntimeError(f"Projection disagrees with oracle: {CASE}, {ERROR}, {QUANTIZATION}")
        ROUNDTRIP = GEOD.Inverse(LAT, LON, ACTUAL[2], ACTUAL[3])['s12']
        BASIS0 = np.array(replay_common.ned_basis(math.radians(LAT0), math.radians(LON0)))
        ORIGIN = np.array(replay_common.ecef(math.radians(LAT0), math.radians(LON0), 0.))
        for HEIGHT in [-100., 0., 500., 3000.]:
            ENU = TOPO.transform(LON, LAT, HEIGHT)
            ORACLE_NED = np.array([ENU[1], ENU[0], -ENU[2]])
            DELTA = np.array(replay_common.ecef(math.radians(LAT), math.radians(LON), HEIGHT)) - ORIGIN
            REPLAY = BASIS0 @ DELTA
            REPLAY_MAX = max(REPLAY_MAX, float(np.linalg.norm(REPLAY-ORACLE_NED)))
            HEIGHT_COMPARISONS += 1
            if HEIGHT == 0:
                TANGENT = ORACLE_NED
        BASIS1 = np.array(replay_common.ned_basis(math.radians(LAT), math.radians(LON)))
        T = BASIS0 @ BASIS1.T
        TILT = math.degrees(math.atan2(np.linalg.norm(T[:2, 2]), T[2, 2]))
        FULL_ANGLE = math.degrees(2 * math.asin(min(1., np.linalg.norm(T-np.eye(3), 2)/2)))
        # Jacobian of independent spherical AEQD wrt physical N/E metres on WGS84.
        # Double PROJ coordinates avoid differentiating PX4's float quantization.
        J = np.zeros((2, 2))
        for AXIS, BEARING in enumerate([0., 90.]):
            PLUS = GEOD.Direct(LAT, LON, BEARING, 1.)
            MINUS = GEOD.Direct(LAT, LON, BEARING+180., 1.)
            EP, NP = AEQD.transform(PLUS['lon2'], PLUS['lat2'])
            EM, NM = AEQD.transform(MINUS['lon2'], MINUS['lat2'])
            J[:, AXIS] = [(NP-NM)/2, (EP-EM)/2]
        ROWS.append(dict(latitude0=LAT0, longitude0=LON0, distance_m=DISTANCE, azimuth_deg=AZIMUTH,
                         latitude=LAT, longitude=LON, raw_reproject_longitude_deg=float(ACTUAL[3]), px4_vs_spherical_oracle_m=float(ERROR), roundtrip_m=ROUNDTRIP,
                         px4_vs_wgs84_tangent_xy_m=float(np.linalg.norm(ACTUAL_NED-TANGENT[:2])),
                         tangent_down_at_equal_ellipsoid_height_m=float(TANGENT[2]),
                         ned_down_angle_deg=TILT, ned_full_frame_angle_deg=FULL_ANGLE,
                         local_velocity_unrotated_error_at_20m_s=float(20*np.linalg.norm(T-np.eye(3), 2)),
                         projected_position_derivative_vs_raw_velocity_at_20m_s=float(
                             20*np.linalg.norm(J-np.eye(2), 2)),
                         fixed_gravity_direction_difference_m_s2=float(np.linalg.norm(
                             T@np.array([0., 0., 9.80665])-np.array([0., 0., 9.80665]))),
                         projection_jacobian=J.tolist()))
    if REPLAY_MAX >= 1e-7:
        raise RuntimeError(f"Replay ECEF/NED disagrees with WGS84 oracle: {REPLAY_MAX}")
    return ROWS, HEIGHT_COMPARISONS, REPLAY_MAX


def rotation(VECTOR):
    VECTOR = np.asarray(VECTOR, dtype=float)
    ANGLE = np.linalg.norm(VECTOR)
    if ANGLE == 0:
        return np.eye(3)
    X, Y, Z = VECTOR / ANGLE
    HAT = np.array([[0., -Z, Y], [Z, 0., -X], [-Y, X, 0.]])
    return np.eye(3) + math.sin(ANGLE)*HAT + (1-math.cos(ANGLE))*(HAT@HAT)


def heading_variances():
    HEADING = []
    P = np.diag([0.01, 0.04, 0.001])
    for ROLL, PITCH in [(0., 0.), (90., 0.), (0., 60.), (90., 60.), (90., 80.)]:
        R = (rotation([0., 0., 0.4]) @ rotation([0., math.radians(PITCH), 0.])
             @ rotation([math.radians(ROLL), 0., 0.]))
        EPS = 1e-6
        NUMERICAL = []
        for AXIS in range(3):
            STEP = np.zeros(3)
            STEP[AXIS] = EPS
            PLUS = R@rotation(STEP)
            MINUS = R@rotation(-STEP)
            DIFFERENCE = math.atan2(PLUS[1, 0], PLUS[0, 0])-math.atan2(MINUS[1, 0], MINUS[0, 0])
            NUMERICAL.append(math.atan2(math.sin(DIFFERENCE), math.cos(DIFFERENCE))/(2*EPS))
        J = np.array([0., math.sin(math.radians(ROLL))/math.cos(math.radians(PITCH)),
                     math.cos(math.radians(ROLL))/math.cos(math.radians(PITCH))])
        if np.max(np.abs(J-np.array(NUMERICAL))) >= 1e-8:
            raise RuntimeError("Euler-heading Jacobian disagrees with numerical perturbation")
        HEADING.append(dict(roll_deg=ROLL, pitch_deg=PITCH, old_body_z_proxy=float(P[2, 2]),
                            px4_style_navigation_down_rotation_variance=float((R@P@R.T)[2, 2]),
                            euler_yaw_linearized_variance=float(J@P@J), right_error_yaw_jacobian=J.tolist()))
    return HEADING


def summarize(OUT, CASES, ROWS, HEIGHT_COMPARISONS, REPLAY_MAX, HEADING):
    IDENTIFIERS = {'latitude0', 'longitude0', 'distance_m', 'azimuth_deg', 'latitude', 'longitude',
                   'projection_jacobian', 'raw_reproject_longitude_deg'}
    NUMERICAL_FIELDS = [K for K in ROWS[0] if K not in IDENTIFIERS]
    SUMMARY = []
    for DISTANCE in DISTANCES:
        GROUP = [R for R in ROWS if R['distance_m'] == DISTANCE and abs(R['latitude0']) <= 60.]
        MAXIMA = {K: max(abs(R[K]) for R in GROUP) for K in NUMERICAL_FIELDS}
        SUMMARY.append(dict(distance_m=DISTANCE, cases=len(GROUP), **MAXIMA))
    with (OUT/'samples.csv').open('w', newline='') as F:
        W = csv.DictWriter(F, fieldnames=list(ROWS[0]))
        W.writeheader()
        W.writerows(ROWS)
    RESULT = dict(scope='Finite geographic grid, not a global error proof or flight accuracy validation',
                  dependencies={'geographiclib': geographiclib.__version__, 'pyproj': pyproj.__version__,
                                'proj': pyproj.proj_version_str, 'numpy': np.__version__},
                  projection_cases=len(CASES), replay_height_comparisons=HEIGHT_COMPARISONS, replay_max_error_m=REPLAY_MAX,
                  raw_reproject_longitude_out_of_range=sum(
                      abs(R['raw_reproject_longitude_deg']) > 180 for R in ROWS),
                  max_projection_oracle_error_m=max(R['px4_vs_spherical_oracle_m'] for R in ROWS),
                  max_roundtrip_error_m=max(R['roundtrip_m'] for R in ROWS),
                  summary_latitude_at_most_60=SUMMARY, heading_variance=HEADING,
                  extreme_latitude_examples=[R for R in ROWS if R['latitude0'] in [
                      80., 89.9] and R['longitude0'] == 0 and R['azimuth_deg'] == 90 and R['distance_m'] in [1000., 10000., 100000.]],
                  taiwan_examples=[R for R in ROWS if R['latitude0'] == 25 and R['longitude0'] == 120 and R['distance_m'] in [1000., 10000.] and R['azimuth_deg'] in [0, 90]])
    (OUT/'analysis.json').write_text(json.dumps(RESULT, indent=2)+'\n')
    return RESULT


def write_manifest(PX4, OUT, RESULT):
    SOURCES = [Path(__file__).resolve(), REPO / "integrations/px4/tests/ProjectionProbe.cpp", REPO / "examples/replay_common.py",
               REPO / "integrations/px4/requirements.txt",
               PX4 / "src/lib/geo/geo.cpp", PX4 / "src/lib/geo/geo.h",
               PX4 / "src/modules/ekf2/EKF/ekf.cpp", PX4 / "src/modules/ekf2/EKF/ekf_helper.cpp",
               PX4 / "build/px4_sitl_formal_eskf/compile_commands.json",
               PX4 / "build/px4_sitl_formal_eskf/src/lib/geo/libgeo.a"]
    SOURCES += sorted((REPO / "integrations/px4/module").glob("FormalEskf*.cpp"))
    SOURCES += [REPO / "integrations/px4/module" / NAME for NAME in
                ("FormalEskf.hpp", "MagneticModel.hpp", "DeploymentProfile.hpp")]
    SOURCES += [REPO / "integrations/px4/tests/ModuleTest.cpp"]
    SOURCES += [REPO / "docs/spec/requirements" / NAME for NAME in
                ("core/state.md", "integration/px4/output-frames.md",
                 "integration/px4/magnetic-model.md", "integration/replay/gnss-quality.md")]
    MANIFEST = {"formal_eskf_head": subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True).strip(),
                "px4_head": subprocess.check_output(["git", "-C", str(PX4), "rev-parse", "HEAD"], text=True).strip(),
                "source_sha256": {str(P): sha256(P) for P in SOURCES},
                "artifact_sha256": {P.name: sha256(P) for P in sorted(OUT.iterdir()) if P.is_file() and P.name in
                                    ("analysis.json", "samples.csv", "probe-input.txt", "probe-output.txt",
                                     "projection_probe.cpp", "projection_probe", "compile-command.json", "compile.log")},
                "dependencies": RESULT["dependencies"],
                "scope": RESULT["scope"]}
    (OUT / "manifest.json").write_text(json.dumps(MANIFEST, indent=2)+"\n")


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("px4", type=Path, help="PX4 checkout with the formal_eskf SITL geo library built")
    PARSER.add_argument("--output", type=Path, default=Path("output/px4-frame-envelope"))
    ARGS = PARSER.parse_args()
    PX4, OUT = ARGS.px4.resolve(), ARGS.output.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    PROBE = compile_probe(PX4, OUT)
    CASES, ORACLES = geographic_cases()
    INPUT = "".join(f"{C[0]:.17g} {C[1]:.17g} {C[4]:.17g} {C[5]:.17g}\n" for C in CASES)
    (OUT / "probe-input.txt").write_text(INPUT)
    RUN = subprocess.run([str(PROBE)], input=INPUT, text=True, capture_output=True, check=True)
    (OUT / "probe-output.txt").write_text(RUN.stdout)
    VALUES = np.array([[float(V) for V in LINE.split()] for LINE in RUN.stdout.splitlines()])
    if VALUES.shape != (len(CASES), 4) or not np.isfinite(VALUES).all():
        raise RuntimeError("PX4 probe returned incomplete or non-finite results")
    ROWS, HEIGHT_COMPARISONS, REPLAY_MAX = compare(CASES, ORACLES, VALUES)
    RESULT = summarize(OUT, CASES, ROWS, HEIGHT_COMPARISONS, REPLAY_MAX, heading_variances())
    write_manifest(PX4, OUT, RESULT)
    print(
        f"Projection: {len(CASES)} points checked against spherical PROJ; round-trip max {RESULT['max_roundtrip_error_m']:.6f} m")
    print(f"Replay ECEF/NED: {HEIGHT_COMPARISONS} WGS84 comparisons; max {REPLAY_MAX:.3g} m")
    print("Frame/model differences recorded; this is not flight accuracy or a global bound.")
    print(f"Report: {OUT / 'analysis.json'}")


if __name__ == "__main__":
    main()
