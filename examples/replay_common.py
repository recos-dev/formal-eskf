"""Shared CSV, coordinate and plotting utilities for the host replay demos."""

import csv
import math


IMU_HEADER = "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz"
NAVIGATION_HEADER = "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy"
INITIAL_HEADER = "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy"
PROGRESS_FORMAT = "{desc}: {bar} {n_fmt}/{total_fmt} stages [{elapsed}]"


def ecef(LAT, LON, ALT):
    """WGS84; latitude/longitude in radians, ellipsoidal altitude in metres."""
    A = 6378137.0
    E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)
    N = A / math.sqrt(1.0 - E2 * math.sin(LAT) ** 2)
    return [(N + ALT) * math.cos(LAT) * math.cos(LON),
            (N + ALT) * math.cos(LAT) * math.sin(LON),
            (N * (1.0 - E2) + ALT) * math.sin(LAT)]


def ned_basis(LAT, LON):
    return [[-math.sin(LAT) * math.cos(LON), -math.sin(LAT) * math.sin(LON), math.cos(LAT)],
            [-math.sin(LON), math.cos(LON), 0.0],
            [-math.cos(LAT) * math.cos(LON), -math.cos(LAT) * math.sin(LON), -math.sin(LAT)]]


def dot(A, B):
    return sum(X * Y for X, Y in zip(A, B))


def write_csv(PATH, HEADER, ROWS):
    with PATH.open("x", newline="") as STREAM:
        WRITER = csv.writer(STREAM)
        WRITER.writerow(HEADER.split(","))
        WRITER.writerows(ROWS)


def plot_replay(OUTPUT, SOURCE_LABEL="GNSS"):
    """Adapt the original plot.py views to this demo's NED/quaternion CSVs."""
    import matplotlib.pyplot as PLOT
    import numpy as NP
    from tqdm import tqdm as TQDM

    with TQDM(total=9, desc="Read plot CSV", dynamic_ncols=True, bar_format=PROGRESS_FORMAT) as PROGRESS:
        STATE = NP.genfromtxt(OUTPUT / "result/state.csv", delimiter=",", names=True, ndmin=1)
        NAVIGATION = NP.genfromtxt(OUTPUT / "input/navigation.csv", delimiter=",", names=True, ndmin=1)
        START = STATE["timestamp_us"][0]
        TIME = (STATE["timestamp_us"] - START) * 1e-6
        NAVIGATION_TIME = (NAVIGATION["timestamp_us"] - START) * 1e-6
        FIGURES = []

        PROGRESS.update(1)
        PROGRESS.set_description_str("Build position / velocity")
        FIGURE, AXES = PLOT.subplots(3, 2, sharex=True, figsize=(11, 8), constrained_layout=True)
        FIGURE.suptitle(f"Position / velocity (fixed local NED) - {SOURCE_LABEL} inputs, not ground truth")
        for ROW, (AXIS, LABEL) in enumerate(zip("ned", ("North", "East", "Down"))):
            for COLUMN, (PREFIX, UNIT) in enumerate((("p", "m"), ("v", "m/s"))):
                AX = AXES[ROW, COLUMN]
                AX.plot(TIME, STATE[PREFIX + AXIS], label="ESKF", linewidth=1)
                AX.plot(NAVIGATION_TIME, NAVIGATION[PREFIX + AXIS], ".", label=SOURCE_LABEL, markersize=2)
                AX.set_ylabel(f"{LABEL} [{UNIT}]")
                AX.grid(True, alpha=0.3)
        for COLUMN, TITLE in enumerate(("Position", "Velocity")):
            AXES[0, COLUMN].set_title(TITLE)
            AXES[0, COLUMN].legend()
            AXES[2, COLUMN].set_xlabel("Time since replay start [s]")
        FIGURES.append(("position_velocity", FIGURE))

        PROGRESS.update(1)
        PROGRESS.set_description_str("Build attitude")
        # Scalar-first q_nb; display ZYX Euler angles without changing the state.
        Q0, Q1, Q2, Q3 = (STATE[KEY] for KEY in ("q0", "q1", "q2", "q3"))
        ROLL = NP.arctan2(2 * (Q0 * Q1 + Q2 * Q3), 1 - 2 * (Q1 * Q1 + Q2 * Q2))
        PITCH = NP.arcsin(NP.clip(2 * (Q0 * Q2 - Q3 * Q1), -1, 1))
        YAW = NP.unwrap(NP.arctan2(2 * (Q0 * Q3 + Q1 * Q2), 1 - 2 * (Q2 * Q2 + Q3 * Q3)))
        FIGURE, AXES = PLOT.subplots(3, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
        FIGURE.suptitle("Estimated attitude - no independent attitude reference")
        for AX, VALUES, LABEL in zip(AXES, (ROLL, PITCH, YAW), ("Roll", "Pitch", "Yaw (unwrapped)")):
            AX.plot(TIME, NP.degrees(VALUES), linewidth=1)
            AX.set_ylabel(f"{LABEL} [deg]")
            AX.grid(True, alpha=0.3)
        AXES[-1].set_xlabel("Time since replay start [s]")
        FIGURES.append(("attitude", FIGURE))

        PROGRESS.update(1)
        PROGRESS.set_description_str("Build trajectory")
        FIGURE, AX = PLOT.subplots(figsize=(7, 7), constrained_layout=True)
        AX.plot(STATE["pe"], STATE["pn"], label="ESKF", linewidth=1)
        AX.plot(NAVIGATION["pe"], NAVIGATION["pn"], ".", label=SOURCE_LABEL, markersize=2)
        AX.set(title=f"Horizontal trajectory - {SOURCE_LABEL} inputs, not ground truth", xlabel="East [m]", ylabel="North [m]")
        AX.set_aspect("equal", adjustable="datalim")
        AX.grid(True, alpha=0.3)
        AX.legend()
        FIGURES.append(("trajectory", FIGURE))

        PROGRESS.update(1)
        PROGRESS.set_description_str("Build 3D trajectory")
        FIGURE, AX = PLOT.subplots(figsize=(9, 8), subplot_kw={"projection": "3d"}, constrained_layout=True)
        # Equal metre scales on all axes, including stationary/planar trajectories.
        # Set limits before plotting: our explicit bounds need no 3D autoscaling,
        # whose internals are incompatible across some Matplotlib installations.
        POINTS = NP.vstack((NP.column_stack((STATE["pe"], STATE["pn"], -STATE["pd"])),
                            NP.column_stack((NAVIGATION["pe"], NAVIGATION["pn"], -NAVIGATION["pd"]))))
        LOWER, UPPER = POINTS.min(axis=0), POINTS.max(axis=0)
        CENTER = (LOWER + UPPER) / 2
        RADIUS = max(float(NP.max(UPPER - LOWER)) * 0.55, 0.5)
        AX.set(xlim=(CENTER[0] - RADIUS, CENTER[0] + RADIUS),
               ylim=(CENTER[1] - RADIUS, CENTER[1] + RADIUS),
               zlim=(CENTER[2] - RADIUS, CENTER[2] + RADIUS), autoscale_on=False)
        AX.set_box_aspect((1, 1, 1))
        # Display ENU (East, North, Up) so height increases upward; stored data stays NED.
        AX.plot(STATE["pe"], STATE["pn"], -STATE["pd"], label="ESKF", linewidth=1)
        AX.plot(NAVIGATION["pe"], NAVIGATION["pn"], -NAVIGATION["pd"], ".", label=SOURCE_LABEL, markersize=2)
        AX.set(title=f"3D trajectory - {SOURCE_LABEL} inputs, not ground truth",
               xlabel="East [m]", ylabel="North [m]", zlabel="Up (-Down) [m]")
        AX.legend()
        FIGURES.append(("trajectory_3d", FIGURE))

        PROGRESS.update(1)
        PLOT_DIR = OUTPUT / "plots"
        PLOT_DIR.mkdir(exist_ok=False)
        for NAME, FIGURE in FIGURES:
            PROGRESS.set_description_str(f"Save {NAME}.png")
            FIGURE.savefig(PLOT_DIR / f"{NAME}.png", dpi=150)
            PROGRESS.update(1)
        PROGRESS.set_description_str("Plots complete")
    print(f"Plots: {PLOT_DIR}", flush=True)
    if PLOT.get_backend().lower() != "agg":
        print("Displaying plots; close the plot windows to finish.", flush=True)
        PLOT.show()
