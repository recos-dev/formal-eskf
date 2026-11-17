#!/usr/bin/env python3
"""Run PX4 SIH, normal arming, offboard square and auto-land; preserve evidence."""
import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from pymavlink import mavutil


GNSS_RECOVERY = ("gnss_quality_recovery", "gnss_restart", "gnss_after_mag_restart", "gnss_height_recovery")


class Flight:
    def __init__(SELF, ARGS):
        SELF.args = ARGS
        SELF.build = ARGS.px4 / "build/px4_sitl_formal_eskf"
        SELF.output = ARGS.output.resolve()
        SELF.output.mkdir(parents=True, exist_ok=False)
        SELF.rootfs = SELF.output / "rootfs"
        SELF.rootfs.mkdir()
        SELF.connection = None
        SELF.target = ARGS.instance + 1
        SELF.messages = {}
        SELF.acks = {}
        SELF.last_heartbeat_s = 0.
        SELF.last_setpoint_s = 0.
        SELF.setpoint = None
        SELF.events = []
        SELF.samples = []
        SELF.started = time.monotonic()
        SELF.process = None

    def event(SELF, TEXT):
        ENTRY = {"wall_s": round(time.monotonic() - SELF.started, 3), "event": TEXT}
        SELF.events.append(ENTRY)
        print(json.dumps(ENTRY), flush=True)

    def cli(SELF, MODULE, *ARGS):
        RESULT = subprocess.run(
            [str(SELF.build / "bin" / f"px4-{MODULE}"), "--instance", str(SELF.args.instance), *ARGS],
            capture_output=True, text=True, timeout=10)
        return {"command": [MODULE, *ARGS], "returncode": RESULT.returncode,
                "output": RESULT.stdout + RESULT.stderr}

    def pump(SELF, DURATION):
        END = time.monotonic() + DURATION
        while time.monotonic() < END:
            NOW = time.monotonic()
            if SELF.process and SELF.process.poll() is not None:
                raise RuntimeError(f"PX4 exited early ({SELF.process.returncode})")
            if NOW - SELF.last_heartbeat_s > 0.5:
                SELF.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                SELF.last_heartbeat_s = NOW
            if SELF.setpoint is not None and NOW - SELF.last_setpoint_s >= 0.05:
                X, Y, Z, YAW = SELF.setpoint
                # position + yaw, with velocity/acceleration/yaw-rate ignored
                MASK = 0b100111111000
                SELF.connection.mav.set_position_target_local_ned_send(
                    int((NOW - SELF.started) * 1000), SELF.target, 1,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK,
                    X, Y, Z, 0., 0., 0., 0., 0., 0., YAW, 0.)
                SELF.last_setpoint_s = NOW
            MSG = SELF.connection.recv_match(blocking=True, timeout=0.02)
            if MSG is None or MSG.get_srcSystem() != SELF.target:
                continue
            KIND = MSG.get_type()
            SELF.messages[KIND] = (NOW, MSG)
            if KIND == "COMMAND_ACK":
                SELF.acks[MSG.command] = MSG.result
            elif KIND == "STATUSTEXT":
                SELF.event(f"PX4: {MSG.text}")
            elif KIND == "LOCAL_POSITION_NED":
                SELF.samples.append({"wall_s": NOW - SELF.started, "time_boot_ms": MSG.time_boot_ms,
                                     "x": MSG.x, "y": MSG.y, "z": MSG.z,
                                     "vx": MSG.vx, "vy": MSG.vy, "vz": MSG.vz,
                                     "setpoint": SELF.setpoint})

    def command(SELF, COMMAND, PARAMS):
        SELF.acks.pop(COMMAND, None)
        PARAMS = list(PARAMS) + [0.] * (7 - len(PARAMS))
        for ATTEMPT in range(3):
            SELF.connection.mav.command_long_send(SELF.target, 1, COMMAND, ATTEMPT, *PARAMS)
            END = time.monotonic() + 3
            while time.monotonic() < END:
                SELF.pump(0.05)
                if COMMAND in SELF.acks:
                    RESULT = SELF.acks.pop(COMMAND)
                    if RESULT != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        raise RuntimeError(f"Command {COMMAND} rejected: MAV_RESULT={RESULT}")
                    SELF.event(f"command {COMMAND} accepted")
                    return
        raise RuntimeError(f"No ACK for command {COMMAND}")

    def wait_for(SELF, PREDICATE, TIMEOUT, DESCRIPTION):
        END = time.monotonic() + TIMEOUT
        while time.monotonic() < END:
            SELF.pump(0.1)
            if PREDICATE():
                SELF.event(DESCRIPTION)
                return
        raise RuntimeError(f"Timed out: {DESCRIPTION}")

    def armed(SELF):
        ITEM = SELF.messages.get("HEARTBEAT")
        return bool(ITEM and ITEM[1].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def pump_simulation(SELF, DURATION):
        """Sensor expiry uses the SIH clock, which can pause on a busy host."""
        START = SELF.messages["LOCAL_POSITION_NED"][1].time_boot_ms
        TARGET = START + round(DURATION * 1000)
        SELF.wait_for(lambda: SELF.messages["LOCAL_POSITION_NED"][1].time_boot_ms >= TARGET,
                      max(30., 5. * DURATION), f"Simulation advanced {DURATION:g} s")
        return {"start_ms": START, "end_ms": SELF.messages["LOCAL_POSITION_NED"][1].time_boot_ms}

    def phase(SELF, NAME, POINT, DURATION):
        SELF.setpoint = POINT
        SELF.event(f"{NAME}: NED/yaw setpoint {POINT}")
        START = time.monotonic()
        ERRORS = []
        while time.monotonic() - START < DURATION:
            SELF.pump(0.1)
            HB_TIME, HB = SELF.messages["HEARTBEAT"]
            if time.monotonic() - HB_TIME > 3 or not SELF.armed():
                raise RuntimeError(f"Heartbeat stale or disarmed during {NAME}")
            if ((HB.custom_mode >> 16) & 0xff) != 6:
                raise RuntimeError(f"Left OFFBOARD during {NAME}: {HB.custom_mode}")
            ITEM = SELF.messages.get("LOCAL_POSITION_NED")
            if not ITEM or time.monotonic() - ITEM[0] > 1:
                raise RuntimeError("Local position telemetry stale")
            P = ITEM[1]
            if not all(math.isfinite(V) for V in (P.x, P.y, P.z, P.vx, P.vy, P.vz)):
                raise RuntimeError("Non-finite local state")
            if abs(P.x) > 30 or abs(P.y) > 30 or P.z < -15 or P.z > 3:
                raise RuntimeError("Flight exceeded test envelope")
            if time.monotonic() - START > DURATION - 3:
                ERRORS.append(math.sqrt((P.x - POINT[0])**2 + (P.y - POINT[1])**2 + (P.z - POINT[2])**2))
        RMS = float(np.sqrt(np.mean(np.square(ERRORS))))
        SELF.event(f"{NAME}: final 3 s position tracking RMSE = {RMS:.3f} m")
        if RMS > 1.0:
            raise RuntimeError(f"Tracking error too high in {NAME}: {RMS:.3f} m")
        return {"phase": NAME, "setpoint": POINT, "duration_s": DURATION, "final_tracking_rmse_m": RMS}

    @staticmethod
    def listener_integer(OUTPUT, NAME):
        MATCH = re.search(rf"^\s*{NAME}:\s+(\d+)", OUTPUT, re.MULTILINE)
        if MATCH is None:
            raise RuntimeError(f"Missing {NAME} in listener output")
        return int(MATCH[1])

    @staticmethod
    def magnetic_model_status(OUTPUT):
        MATCH = re.search(r"mag model: available=(\d+) checks=(\d+) model_revision=(\d+) "
                          r"heading_revision=(\d+) source=(\d+) unavailable=(\d+)", OUTPUT)
        if MATCH is None:
            raise RuntimeError("Missing magnetic model status")
        return dict(zip(("available", "checks", "model_revision", "heading_revision", "source", "unavailable"),
                        map(int, MATCH.groups())))

    @staticmethod
    def check_gravity(NAME, HEIGHT, FLAGS, GRAVITY_ENABLED):
        PASSED = True
        GRAVITY = re.search(r"gravity: enabled=(\d+) tilt=(\d+) active=(\d+) samples=(\d+) fused=(\d+)",
                            HEIGHT["output"])
        if not GRAVITY:
            raise RuntimeError("Missing gravity diagnostics")
        ENABLED, TILT, ACTIVE, SAMPLES, FUSED = map(int, GRAVITY.groups())
        PASSED = PASSED and ENABLED == int(GRAVITY_ENABLED) and TILT == 1 and SAMPLES > 0
        if not GRAVITY_ENABLED:
            PASSED = PASSED and ACTIVE == 0 and FUSED == 0
        elif NAME in ("invalid_gnss_stop", "gnss_timeout", "mag_unaided"):
            PASSED = PASSED and ACTIVE == 1 and FUSED > 0
        elif NAME in GNSS_RECOVERY:
            PASSED = PASSED and ACTIVE == 0
        PASSED = PASSED and f"cs_gravity_vector: {bool(ACTIVE)}" in FLAGS["output"]
        return PASSED

    @staticmethod
    def check_gnss(NAME, LOCAL, STATUS, FLAGS, PREVIOUS_POSITION_RESET, PREVIOUS_VELOCITY_RESET):
        PASSED = True
        POSITION_RESET = Flight.listener_integer(LOCAL["output"], "xy_reset_counter")
        VELOCITY_RESET = Flight.listener_integer(LOCAL["output"], "vxy_reset_counter")
        if NAME in GNSS_RECOVERY:
            PASSED = PASSED and (POSITION_RESET - PREVIOUS_POSITION_RESET) % 256 == 1
            PASSED = PASSED and (VELOCITY_RESET - PREVIOUS_VELOCITY_RESET) % 256 == 1
        else:
            PASSED = PASSED and POSITION_RESET == PREVIOUS_POSITION_RESET
            PASSED = PASSED and VELOCITY_RESET == PREVIOUS_VELOCITY_RESET
        if NAME.startswith("invalid_gnss"):
            # Simulator publishes no fix, zero satellites, bad accuracy and DOP.
            PASSED = PASSED and Flight.listener_integer(STATUS["output"], "gps_check_fail_flags") & 0x3f == 0x3f
        if NAME in ("invalid_gnss_grace", "gnss_gap_grace", "invalid_gnss_invalid", "gnss_gap_invalid"):
            PASSED = PASSED and "cs_gnss_pos: True" in FLAGS["output"] and "cs_gnss_vel: True" in FLAGS["output"]
            PASSED = PASSED and "dead_reckoning: True" in LOCAL["output"]
        if NAME in ("invalid_gnss_stop", "gnss_timeout"):
            PASSED = PASSED and "cs_gnss_pos: False" in FLAGS["output"] and "cs_gnss_vel: False" in FLAGS["output"]
        return PASSED

    @staticmethod
    def check_magnetic(NAME, LOCAL, STATUS, FLAGS, ATTITUDE, PREVIOUS_HEADING_RESET):
        PASSED = True
        HEADING_RESET = Flight.listener_integer(LOCAL["output"], "heading_reset_counter")
        EXPECTED_RESETS = int(NAME == "mag_unaided_restart")
        PASSED = PASSED and (HEADING_RESET - PREVIOUS_HEADING_RESET) % 256 == EXPECTED_RESETS
        PASSED = PASSED and Flight.listener_integer(ATTITUDE["output"], "quat_reset_counter") == HEADING_RESET
        PASSED = PASSED and Flight.listener_integer(STATUS["output"], "reset_count_quat") == HEADING_RESET
        PASSED = PASSED and "cs_mag_3d: False" in FLAGS["output"]
        if NAME.startswith("mag_"):
            ACTIVE = NAME not in ("mag_timeout", "mag_unaided_timeout")
            ALIGNED = NAME != "mag_unaided_timeout"
            PASSED = PASSED and f"cs_mag: {ACTIVE}" in FLAGS["output"]
            PASSED = PASSED and f"cs_mag_hdg: {ACTIVE}" in FLAGS["output"]
            PASSED = PASSED and f"cs_yaw_align: {ALIGNED}" in FLAGS["output"]
            if ACTIVE:
                PASSED = PASSED and "cs_mag_heading_consistent: True" in FLAGS["output"]
        return PASSED

    @staticmethod
    def check_height(NAME, LOCAL, STATUS, GLOBAL, HEIGHT, PREVIOUS_HEIGHT_RESET):
        PASSED = True
        HEIGHT_RESET = Flight.listener_integer(LOCAL["output"], "z_reset_counter")
        PASSED = PASSED and (HEIGHT_RESET - PREVIOUS_HEIGHT_RESET) % 256 == int(NAME in GNSS_RECOVERY)
        PASSED = PASSED and Flight.listener_integer(GLOBAL["output"], "alt_reset_counter") == HEIGHT_RESET
        PASSED = PASSED and Flight.listener_integer(STATUS["output"], "reset_count_pod_d") == HEIGHT_RESET
        if NAME in GNSS_RECOVERY:
            LOCAL_DELTA = float(re.search(r"\bdelta_z:\s+([^\s]+)", LOCAL["output"])[1])
            GLOBAL_DELTA = float(re.search(r"\bdelta_alt:\s+([^\s]+)", GLOBAL["output"])[1])
            PASSED = PASSED and abs(LOCAL_DELTA + GLOBAL_DELTA) < 1e-4
        HEIGHT_EXPECTATIONS = {
            "baro_timeout": "height: ref=1 baro=0 gnss=1",
            "baro_restart": "height: ref=1 baro=1 gnss=1",
            "gnss_height_fallback": "height: ref=0 baro=1 gnss=0",
            "all_height_timeout": "height: ref=2 baro=0 gnss=0",
            "baro_height_recovery": "height: ref=0 baro=1 gnss=0",
            "gnss_height_recovery": "height: ref=1 baro=1 gnss=1",
        }
        if NAME in HEIGHT_EXPECTATIONS:
            PASSED = PASSED and HEIGHT_EXPECTATIONS[NAME] in HEIGHT["output"]
        return PASSED

    def ground_fault_checks(SELF):
        """Exercise actual simulated sensor publishers, with the vehicle disarmed."""
        if SELF.armed():
            raise RuntimeError("Ground fault checks require a disarmed vehicle")
        CHECKS = []
        STEPS = [
            ("invalid_gnss_grace", ("param", "set", "SIM_GPS_USED", "0"), 2., "xy_valid: True"),
            ("invalid_gnss_invalid", ("formal_eskf", "status"), 3.5, "xy_valid: False"),
            ("invalid_gnss_stop", ("formal_eskf", "status"), 3., "xy_valid: False"),
            ("gnss_quality_recovery", ("param", "set", "SIM_GPS_USED", "10"), 4., "xy_valid: True"),
            ("gnss_gap_grace", ("sensor_gps_sim", "stop"), 2., "xy_valid: True"),
            ("gnss_gap_invalid", ("formal_eskf", "status"), 3.5, "xy_valid: False"),
            ("gnss_timeout", ("formal_eskf", "status"), 3., "xy_valid: False"),
            ("gnss_restart", ("sensor_gps_sim", "start"), 4., "xy_valid: True"),
            ("mag_timeout", ("sensor_mag_sim", "stop"), 3., "heading_good_for_control: False"),
            ("mag_restart", ("sensor_mag_sim", "start"), 4., "heading_good_for_control: True"),
            ("mag_unaided", ("sensor_gps_sim", "stop"), 8.5, "xy_valid: False"),
            ("mag_unaided_timeout", ("sensor_mag_sim", "stop"), 1.5, "heading_good_for_control: False"),
            ("mag_unaided_restart", ("sensor_mag_sim", "start"), 2., "heading_good_for_control: True"),
            ("gnss_after_mag_restart", ("sensor_gps_sim", "start"), 4., "xy_valid: True"),
            ("baro_timeout", ("sensor_baro_sim", "stop"), 0.6, "z_valid: True"),
            ("baro_restart", ("sensor_baro_sim", "start"), 2., "z_valid: True"),
            ("gnss_height_fallback", ("sensor_gps_sim", "stop"), 1.2, "z_valid: True"),
            ("all_height_timeout", ("sensor_baro_sim", "stop"), 5.2, "z_valid: False"),
            ("baro_height_recovery", ("sensor_baro_sim", "start"), 2., "z_valid: True"),
            ("gnss_height_recovery", ("sensor_gps_sim", "start"), 4., "xy_valid: True"),
        ]
        BASELINE = SELF.cli("listener", "vehicle_local_position", "-n", "1")["output"]
        PREVIOUS_POSITION_RESET = SELF.listener_integer(BASELINE, "xy_reset_counter")
        PREVIOUS_VELOCITY_RESET = SELF.listener_integer(BASELINE, "vxy_reset_counter")
        PREVIOUS_HEADING_RESET = SELF.listener_integer(BASELINE, "heading_reset_counter")
        PREVIOUS_HEIGHT_RESET = SELF.listener_integer(BASELINE, "z_reset_counter")
        GRAVITY_ENABLED = int(os.environ.get("PX4_PARAM_FESKF_GRAV_EN", "1")) != 0
        try:
            for NAME, COMMAND, DURATION, EXPECTED in STEPS:
                ACTION = SELF.cli(*COMMAND)
                if ACTION["returncode"] != 0:
                    raise RuntimeError(f"Fault injection failed: {ACTION}")
                ELAPSED = SELF.pump_simulation(DURATION)
                LOCAL = SELF.cli("listener", "vehicle_local_position", "-n", "1")
                RECOVERY = []
                if NAME in GNSS_RECOVERY:
                    # A long gap can produce a failing stationary-speed LPF sample.
                    # Wait for real health recovery, never bypass its checks/timer.
                    for ATTEMPT in range(40):
                        DIAGNOSTICS = SELF.cli("formal_eskf", "status")
                        RECOVERY.append({"local": LOCAL, "diagnostics": DIAGNOSTICS})
                        if EXPECTED in LOCAL["output"]:
                            break
                        if SELF.listener_integer(LOCAL["output"], "xy_reset_counter") != PREVIOUS_POSITION_RESET:
                            raise RuntimeError("Position reset occurred before recovery")
                        if SELF.listener_integer(LOCAL["output"], "vxy_reset_counter") != PREVIOUS_VELOCITY_RESET:
                            raise RuntimeError("Velocity reset occurred before recovery")
                        SELF.pump_simulation(0.5)
                        LOCAL = SELF.cli("listener", "vehicle_local_position", "-n", "1")
                STATUS = SELF.cli("listener", "estimator_status", "-n", "1")
                FLAGS = SELF.cli("listener", "estimator_status_flags", "-n", "1")
                ATTITUDE = SELF.cli("listener", "vehicle_attitude", "-n", "1")
                GLOBAL = SELF.cli("listener", "vehicle_global_position", "-n", "1")
                HEIGHT = SELF.cli("formal_eskf", "status")
                RESULTS = {
                    "local_validity": EXPECTED in LOCAL["output"],
                    "gravity": SELF.check_gravity(NAME, HEIGHT, FLAGS, GRAVITY_ENABLED),
                    "gnss": SELF.check_gnss(NAME, LOCAL, STATUS, FLAGS, PREVIOUS_POSITION_RESET, PREVIOUS_VELOCITY_RESET),
                    "magnetic": SELF.check_magnetic(NAME, LOCAL, STATUS, FLAGS, ATTITUDE, PREVIOUS_HEADING_RESET),
                    "height": SELF.check_height(NAME, LOCAL, STATUS, GLOBAL, HEIGHT, PREVIOUS_HEIGHT_RESET),
                }
                PASSED = all(RESULTS.values())
                PREVIOUS_POSITION_RESET = SELF.listener_integer(LOCAL["output"], "xy_reset_counter")
                PREVIOUS_VELOCITY_RESET = SELF.listener_integer(LOCAL["output"], "vxy_reset_counter")
                PREVIOUS_HEADING_RESET = SELF.listener_integer(LOCAL["output"], "heading_reset_counter")
                PREVIOUS_HEIGHT_RESET = SELF.listener_integer(LOCAL["output"], "z_reset_counter")
                CHECKS.append({"name": NAME, "pass": PASSED, "checks": RESULTS, "expected": EXPECTED, "simulation_time": ELAPSED,
                               "action": ACTION, "local": LOCAL, "status": STATUS, "flags": FLAGS, "attitude": ATTITUDE,
                               "recovery": RECOVERY, "global": GLOBAL, "height": HEIGHT})
                SELF.event(f"ground fault check {NAME}: {'PASS' if PASSED else 'FAIL'}")
                if not PASSED:
                    raise RuntimeError(f"Sensor state, diagnostics or reset assertion failed after {NAME}")
        finally:
            (SELF.output / "faults.json").write_text(json.dumps(CHECKS, indent=2))
        return CHECKS

    def await_startup(SELF, ENV):
        # Startup topic preparation publishes a zero-epoch invalid sample.
        SELF.wait_for(lambda: "LOCAL_POSITION_NED" in SELF.messages and
                      SELF.messages["LOCAL_POSITION_NED"][1].time_boot_ms > 0 and "HEARTBEAT" in SELF.messages,
                      SELF.args.startup_timeout, "Estimator telemetry available")
        SELF.pump(10.)
        STARTUP = [SELF.cli("formal_eskf", "status"), SELF.cli("listener", "vehicle_local_position", "-n", "1"),
                   SELF.cli("listener", "estimator_status", "-n", "1"),
                   SELF.cli("commander", "status"), SELF.cli("uorb", "top", "-1")]
        (SELF.output / "startup.json").write_text(json.dumps(STARTUP, indent=2))
        if "initialized=1 fault=0" not in STARTUP[0]["output"]:
            raise RuntimeError("formal-eskf did not initialize cleanly")
        MODEL_INITIAL = SELF.magnetic_model_status(STARTUP[0]["output"])
        DECL_TYPE = int(ENV.get("PX4_PARAM_FESKF_DECL_TYPE", "3"))
        SAVED_DECL = float(ENV.get("PX4_PARAM_FESKF_MAG_DECL", "0"))
        EXPECTED_SOURCE = 2 if DECL_TYPE & 1 else 1 if DECL_TYPE & 2 and SAVED_DECL != 0 else 0
        if MODEL_INITIAL["available"] != 1 or MODEL_INITIAL["source"] != EXPECTED_SOURCE:
            raise RuntimeError("WMM availability or configured heading source mismatch")
        if (SELF.build / "bin/px4-ekf2").exists():
            raise RuntimeError("POC build unexpectedly contains EKF2")
        return MODEL_INITIAL, DECL_TYPE, SAVED_DECL, EXPECTED_SOURCE

    def fly_mission(SELF, RESULT):
        SELF.setpoint = (0., 0., -5., 0.)
        SELF.pump(2.)
        SELF.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1., 6., 0.])
        SELF.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1., 0.])
        SELF.wait_for(SELF.armed, 5, "Normally armed (no force flag)")
        SELF.wait_for(lambda: ((SELF.messages["HEARTBEAT"][1].custom_mode >> 16) & 0xff) == 6,
                      5, "OFFBOARD active")
        POINTS = [("takeoff_hover", (0., 0., -5., 0.), 18.),
                  ("north", (8., 0., -5., 0.), 12.),
                  ("east_yaw", (8., 8., -5., math.pi / 4), 12.),
                  ("south", (0., 8., -5., math.pi / 4), 12.),
                  ("return_hover", (0., 0., -5., 0.), 12.)]
        for NAME, POINT, DURATION in POINTS:
            RESULT["phases"].append(SELF.phase(NAME, POINT, DURATION))
        SELF.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [0., 0., 0., math.nan, math.nan, math.nan, math.nan])
        SELF.setpoint = None
        SELF.wait_for(lambda: not SELF.armed(), 45, "Auto-land completed and automatically disarmed")
        SELF.pump(2.)

    def check_final_state(SELF, RESULT, MODEL_INITIAL, DECL_TYPE, SAVED_DECL, EXPECTED_SOURCE):
        RESULT["final_status"] = SELF.cli("formal_eskf", "status")
        if "fault=0" not in RESULT["final_status"]["output"] or "estimator_errors=0" not in RESULT["final_status"]["output"]:
            raise RuntimeError("Estimator reported a numerical failure")
        MODEL_FINAL = SELF.magnetic_model_status(RESULT["final_status"]["output"])
        RESULT["magnetic_model"] = {"initial": MODEL_INITIAL, "final": MODEL_FINAL,
                                    "declination_type": DECL_TYPE, "saved_declination_deg": SAVED_DECL}
        if MODEL_FINAL["checks"] <= MODEL_INITIAL["checks"] or MODEL_FINAL["source"] != EXPECTED_SOURCE:
            raise RuntimeError("Periodic WMM lookup did not advance or rewrote heading-source selection")
        if not DECL_TYPE & 1 and MODEL_FINAL["heading_revision"] != MODEL_INITIAL["heading_revision"]:
            raise RuntimeError("Health-only WMM lookup changed the observation reference")

    def shutdown(SELF, RESULT):
        """Reap only this run's process, even when the PX4 shutdown command fails."""
        if SELF.process is None or SELF.process.poll() is not None:
            return
        try:
            RESULT["shutdown"] = SELF.cli("shutdown")
        except (OSError, subprocess.TimeoutExpired) as ERROR:
            RESULT["shutdown_error"] = str(ERROR)
        try:
            SELF.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(SELF.process.pid, signal.SIGTERM)
            try:
                SELF.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(SELF.process.pid, signal.SIGKILL)
                SELF.process.wait(timeout=5)

    def run(SELF):
        ENV = os.environ.copy()
        ENV.update(PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim",
                   PX4_PARAM_FESKF_EN="1", PX4_PARAM_SENS_IMU_MODE="1",
                   PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_SDLOG_MODE="1",
                   PX4_PARAM_SDLOG_PROFILE="3")
        RESULT = {"result": "fail", "simulator": "PX4 SIH quadx", "instance": SELF.args.instance,
                  "normal_arming": True, "phases": []}
        PROVENANCE = {
            "px4_commit": subprocess.check_output(["git", "-C", str(SELF.args.px4), "rev-parse", "HEAD"], text=True).strip(),
            "package": json.loads((SELF.args.px4 / "src/modules/formal_eskf/package-manifest.json").read_text()),
            "binary_sha256": hashlib.sha256((SELF.build / "bin/px4").read_bytes()).hexdigest(),
            "startup_timeout_s": SELF.args.startup_timeout,
            "board_config": (SELF.build / "px4_boardconfig.h").read_text(),
            "px4_environment": {KEY: VALUE for KEY, VALUE in ENV.items() if KEY.startswith("PX4_")},
        }
        (SELF.output / "provenance.json").write_text(json.dumps(PROVENANCE, indent=2))
        FAILURE = None
        CONSOLE = (SELF.output / "px4.log").open("w")
        try:
            SELF.connection = mavutil.mavlink_connection(
                f"udpin:127.0.0.1:{14540 + SELF.args.instance}", source_system=250, source_component=190)
            SELF.process = subprocess.Popen(
                [str(SELF.build / "bin/px4"), "-d", "-i", str(SELF.args.instance),
                 "-w", str(SELF.rootfs), str(SELF.build / "etc")],
                cwd=SELF.args.px4, env=ENV, stdout=CONSOLE, stderr=subprocess.STDOUT,
                start_new_session=True)
            MODEL_INITIAL, DECL_TYPE, SAVED_DECL, EXPECTED_SOURCE = SELF.await_startup(ENV)
            SELF.fly_mission(RESULT)
            SELF.check_final_state(RESULT, MODEL_INITIAL, DECL_TYPE, SAVED_DECL, EXPECTED_SOURCE)
            if SELF.args.check_faults:
                RESULT["ground_fault_checks"] = SELF.ground_fault_checks()
            RESULT["result"] = "flight_pass_pending_ulog_analysis"
        except Exception as ERROR:
            FAILURE = ERROR
            RESULT["error"] = str(ERROR)
            SELF.event(f"FAIL: {ERROR}")
            if SELF.process and SELF.process.poll() is None:
                RESULT["failure_status"] = [SELF.cli("formal_eskf", "status"), SELF.cli("commander", "status"),
                                            SELF.cli("simulator_sih", "status"), SELF.cli("logger", "status"),
                                            SELF.cli("listener", "failsafe_flags", "-n", "1")]
                if SELF.armed():
                    try:
                        SELF.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [0., 0., 0., math.nan, math.nan, math.nan, math.nan])
                        SELF.setpoint = None
                        SELF.wait_for(lambda: not SELF.armed(), 35, "Failure cleanup landed")
                    except Exception as CLEANUP_ERROR:
                        RESULT["cleanup_error"] = str(CLEANUP_ERROR)
        finally:
            try:
                SELF.shutdown(RESULT)
            except (OSError, subprocess.TimeoutExpired) as ERROR:
                RESULT["cleanup_error"] = str(ERROR)
                RESULT["result"] = "fail"
                if FAILURE is None:
                    FAILURE = ERROR
            finally:
                CONSOLE.close()
                if SELF.connection is not None:
                    SELF.connection.close()
            RESULT["events"] = SELF.events
            RESULT["wall_duration_s"] = time.monotonic() - SELF.started
            RESULT["ulogs"] = [str(P.relative_to(SELF.output)) for P in SELF.rootfs.glob("log/**/*.ulg")]
            (SELF.output / "flight.json").write_text(json.dumps(RESULT, indent=2, allow_nan=False))
            (SELF.output / "telemetry.json").write_text(json.dumps(SELF.samples, allow_nan=False))
        if FAILURE:
            raise FAILURE
        return RESULT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--px4", type=Path, required=True, help="PX4 checkout containing the installed and built package")
    PARSER.add_argument("--output", type=Path, required=True, help="New evidence directory")
    PARSER.add_argument("--instance", type=int, default=1, choices=range(1, 10))
    PARSER.add_argument("--startup-timeout", type=float, default=60., help="Seconds allowed for startup on a busy host (default: 60)")
    PARSER.add_argument("--check-faults", action="store_true", help="After landing, test GNSS/magnetometer rejection, timeouts and recovery")
    ARGS = PARSER.parse_args()
    if not math.isfinite(ARGS.startup_timeout) or ARGS.startup_timeout <= 0:
        PARSER.error("--startup-timeout must be finite and positive")
    ARGS.px4 = ARGS.px4.resolve()
    Flight(ARGS).run()


if __name__ == "__main__":
    main()
