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


class Flight:
    def __init__(self, ARGS):
        self.args = ARGS
        self.build = ARGS.px4 / "build/px4_sitl_formal_eskf"
        self.output = ARGS.output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.rootfs = self.output / "rootfs"
        self.rootfs.mkdir()
        self.conn = mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{14540 + ARGS.instance}", source_system=250, source_component=190)
        self.target = ARGS.instance + 1
        self.messages = {}
        self.acks = {}
        self.last_hb = 0.
        self.last_sp = 0.
        self.setpoint = None
        self.events = []
        self.samples = []
        self.started = time.monotonic()
        self.process = None

    def event(self, TEXT):
        ENTRY = {"wall_s": round(time.monotonic() - self.started, 3), "event": TEXT}
        self.events.append(ENTRY)
        print(json.dumps(ENTRY), flush=True)

    def cli(self, MODULE, *ARGS):
        RESULT = subprocess.run(
            [str(self.build / "bin" / f"px4-{MODULE}"), "--instance", str(self.args.instance), *ARGS],
            capture_output=True, text=True, timeout=10)
        return {"command": [MODULE, *ARGS], "returncode": RESULT.returncode,
                "output": RESULT.stdout + RESULT.stderr}

    def pump(self, DURATION):
        END = time.monotonic() + DURATION
        while time.monotonic() < END:
            NOW = time.monotonic()
            if self.process and self.process.poll() is not None:
                raise RuntimeError(f"PX4 exited early ({self.process.returncode})")
            if NOW - self.last_hb > 0.5:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.last_hb = NOW
            if self.setpoint is not None and NOW - self.last_sp >= 0.05:
                X, Y, Z, YAW = self.setpoint
                # position + yaw, with velocity/acceleration/yaw-rate ignored
                MASK = 0b100111111000
                self.conn.mav.set_position_target_local_ned_send(
                    int((NOW - self.started) * 1000), self.target, 1,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK,
                    X, Y, Z, 0., 0., 0., 0., 0., 0., YAW, 0.)
                self.last_sp = NOW
            MSG = self.conn.recv_match(blocking=True, timeout=0.02)
            if MSG is None or MSG.get_srcSystem() != self.target:
                continue
            KIND = MSG.get_type()
            self.messages[KIND] = (NOW, MSG)
            if KIND == "COMMAND_ACK":
                self.acks[MSG.command] = MSG.result
            elif KIND == "STATUSTEXT":
                self.event(f"PX4: {MSG.text}")
            elif KIND == "LOCAL_POSITION_NED":
                self.samples.append({"wall_s": NOW - self.started, "time_boot_ms": MSG.time_boot_ms,
                                     "x": MSG.x, "y": MSG.y, "z": MSG.z,
                                     "vx": MSG.vx, "vy": MSG.vy, "vz": MSG.vz,
                                     "setpoint": self.setpoint})

    def command(self, COMMAND, PARAMS):
        self.acks.pop(COMMAND, None)
        PARAMS = list(PARAMS) + [0.] * (7 - len(PARAMS))
        for ATTEMPT in range(3):
            self.conn.mav.command_long_send(self.target, 1, COMMAND, ATTEMPT, *PARAMS)
            END = time.monotonic() + 3
            while time.monotonic() < END:
                self.pump(0.05)
                if COMMAND in self.acks:
                    RESULT = self.acks.pop(COMMAND)
                    if RESULT != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        raise RuntimeError(f"Command {COMMAND} rejected: MAV_RESULT={RESULT}")
                    self.event(f"command {COMMAND} accepted")
                    return
        raise RuntimeError(f"No ACK for command {COMMAND}")

    def wait_for(self, PREDICATE, TIMEOUT, DESCRIPTION):
        END = time.monotonic() + TIMEOUT
        while time.monotonic() < END:
            self.pump(0.1)
            if PREDICATE():
                self.event(DESCRIPTION)
                return
        raise RuntimeError(f"Timed out: {DESCRIPTION}")

    def armed(self):
        ITEM = self.messages.get("HEARTBEAT")
        return bool(ITEM and ITEM[1].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def pump_simulation(self, DURATION):
        """Sensor expiry uses the SIH clock, which can pause on a busy host."""
        START = self.messages["LOCAL_POSITION_NED"][1].time_boot_ms
        TARGET = START + round(DURATION * 1000)
        self.wait_for(lambda: self.messages["LOCAL_POSITION_NED"][1].time_boot_ms >= TARGET,
                      max(30., 5. * DURATION), f"Simulation advanced {DURATION:g} s")
        return {"start_ms": START, "end_ms": self.messages["LOCAL_POSITION_NED"][1].time_boot_ms}

    def phase(self, NAME, POINT, DURATION):
        self.setpoint = POINT
        self.event(f"{NAME}: NED/yaw setpoint {POINT}")
        START = time.monotonic()
        ERRORS = []
        while time.monotonic() - START < DURATION:
            self.pump(0.1)
            HB_TIME, HB = self.messages["HEARTBEAT"]
            if time.monotonic() - HB_TIME > 3 or not self.armed():
                raise RuntimeError(f"Heartbeat stale or disarmed during {NAME}")
            if ((HB.custom_mode >> 16) & 0xff) != 6:
                raise RuntimeError(f"Left OFFBOARD during {NAME}: {HB.custom_mode}")
            ITEM = self.messages.get("LOCAL_POSITION_NED")
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
        self.event(f"{NAME}: final 3 s position tracking RMSE = {RMS:.3f} m")
        if RMS > 1.0:
            raise RuntimeError(f"Tracking error too high in {NAME}: {RMS:.3f} m")
        return {"phase": NAME, "setpoint": POINT, "duration_s": DURATION, "final_tracking_rmse_m": RMS}

    @staticmethod
    def listener_integer(OUTPUT, NAME):
        MATCH = re.search(rf"^\s*{NAME}:\s+(\d+)", OUTPUT, re.MULTILINE)
        if MATCH is None:
            raise RuntimeError(f"Missing {NAME} in listener output")
        return int(MATCH[1])

    def ground_fault_checks(self):
        """Exercise actual simulated sensor publishers, with the vehicle disarmed."""
        if self.armed():
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
        ]
        BASELINE = self.cli("listener", "vehicle_local_position", "-n", "1")["output"]
        PREVIOUS_POSITION_RESET = self.listener_integer(BASELINE, "xy_reset_counter")
        PREVIOUS_VELOCITY_RESET = self.listener_integer(BASELINE, "vxy_reset_counter")
        try:
            for NAME, COMMAND, DURATION, EXPECTED in STEPS:
                ACTION = self.cli(*COMMAND)
                if ACTION["returncode"] != 0:
                    raise RuntimeError(f"Fault injection failed: {ACTION}")
                ELAPSED = self.pump_simulation(DURATION)
                LOCAL = self.cli("listener", "vehicle_local_position", "-n", "1")
                RECOVERY = []
                if NAME in ("gnss_quality_recovery", "gnss_restart"):
                    # A long gap can produce a failing stationary-speed LPF sample.
                    # Wait for real health recovery, never bypass its checks/timer.
                    for ATTEMPT in range(40):
                        DIAGNOSTICS = self.cli("formal_eskf", "status")
                        RECOVERY.append({"local": LOCAL, "diagnostics": DIAGNOSTICS})
                        if EXPECTED in LOCAL["output"]:
                            break
                        if self.listener_integer(LOCAL["output"], "xy_reset_counter") != PREVIOUS_POSITION_RESET:
                            raise RuntimeError("Position reset occurred before recovery")
                        if self.listener_integer(LOCAL["output"], "vxy_reset_counter") != PREVIOUS_VELOCITY_RESET:
                            raise RuntimeError("Velocity reset occurred before recovery")
                        self.pump_simulation(0.5)
                        LOCAL = self.cli("listener", "vehicle_local_position", "-n", "1")
                STATUS = self.cli("listener", "estimator_status", "-n", "1")
                FLAGS = self.cli("listener", "estimator_status_flags", "-n", "1")
                PASSED = EXPECTED in LOCAL["output"]
                POSITION_RESET = self.listener_integer(LOCAL["output"], "xy_reset_counter")
                VELOCITY_RESET = self.listener_integer(LOCAL["output"], "vxy_reset_counter")
                if NAME in ("gnss_quality_recovery", "gnss_restart"):
                    PASSED = PASSED and (POSITION_RESET - PREVIOUS_POSITION_RESET) % 256 == 1
                    PASSED = PASSED and (VELOCITY_RESET - PREVIOUS_VELOCITY_RESET) % 256 == 1
                else:
                    PASSED = PASSED and POSITION_RESET == PREVIOUS_POSITION_RESET
                    PASSED = PASSED and VELOCITY_RESET == PREVIOUS_VELOCITY_RESET
                if NAME.startswith("invalid_gnss"):
                    # Simulator publishes no fix, zero satellites, bad accuracy and DOP.
                    PASSED = PASSED and self.listener_integer(STATUS["output"], "gps_check_fail_flags") & 0x3f == 0x3f
                if NAME in ("invalid_gnss_grace", "gnss_gap_grace", "invalid_gnss_invalid", "gnss_gap_invalid"):
                    PASSED = PASSED and "cs_gnss_pos: True" in FLAGS["output"] and "cs_gnss_vel: True" in FLAGS["output"]
                    PASSED = PASSED and "dead_reckoning: True" in LOCAL["output"]
                if NAME in ("invalid_gnss_stop", "gnss_timeout"):
                    PASSED = PASSED and "cs_gnss_pos: False" in FLAGS["output"] and "cs_gnss_vel: False" in FLAGS["output"]
                PREVIOUS_POSITION_RESET = POSITION_RESET
                PREVIOUS_VELOCITY_RESET = VELOCITY_RESET
                CHECKS.append({"name": NAME, "pass": PASSED, "expected": EXPECTED, "simulation_time": ELAPSED,
                               "action": ACTION, "local": LOCAL, "status": STATUS, "flags": FLAGS,
                               "recovery": RECOVERY})
                self.event(f"ground fault check {NAME}: {'PASS' if PASSED else 'FAIL'}")
                if not PASSED:
                    raise RuntimeError(f"GNSS/magnetometer state, diagnostics or reset assertion failed after {NAME}")
        finally:
            (self.output / "faults.json").write_text(json.dumps(CHECKS, indent=2))
        return CHECKS

    def run(self):
        ENV = os.environ.copy()
        ENV.update(PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim",
                   PX4_PARAM_FESKF_EN="1", PX4_PARAM_SENS_IMU_MODE="1",
                   PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_SDLOG_MODE="1",
                   PX4_PARAM_SDLOG_PROFILE="3")
        RESULT = {"result": "fail", "simulator": "PX4 SIH quadx", "instance": self.args.instance,
                  "normal_arming": True, "phases": []}
        PROVENANCE = {
            "px4_commit": subprocess.check_output(["git", "-C", str(self.args.px4), "rev-parse", "HEAD"], text=True).strip(),
            "package": json.loads((self.args.px4 / "src/modules/formal_eskf/package-manifest.json").read_text()),
            "binary_sha256": hashlib.sha256((self.build / "bin/px4").read_bytes()).hexdigest(),
            "startup_timeout_s": self.args.startup_timeout,
            "board_config": (self.build / "px4_boardconfig.h").read_text(),
            "px4_environment": {KEY: VALUE for KEY, VALUE in ENV.items() if KEY.startswith("PX4_")},
        }
        (self.output / "provenance.json").write_text(json.dumps(PROVENANCE, indent=2))
        FAILURE = None
        CONSOLE = (self.output / "px4.log").open("w")
        try:
            self.process = subprocess.Popen(
                [str(self.build / "bin/px4"), "-d", "-i", str(self.args.instance),
                 "-w", str(self.rootfs), str(self.build / "etc")],
                cwd=self.args.px4, env=ENV, stdout=CONSOLE, stderr=subprocess.STDOUT,
                start_new_session=True)
            self.wait_for(lambda: "LOCAL_POSITION_NED" in self.messages and "HEARTBEAT" in self.messages,
                          self.args.startup_timeout, "Estimator telemetry available")
            self.pump(10.)
            STARTUP = [self.cli("formal_eskf", "status"), self.cli("listener", "vehicle_local_position", "-n", "1"),
                       self.cli("listener", "estimator_status", "-n", "1"),
                       self.cli("commander", "status"), self.cli("uorb", "top", "-1")]
            (self.output / "startup.json").write_text(json.dumps(STARTUP, indent=2))
            if "initialized=1 fault=0" not in STARTUP[0]["output"]:
                raise RuntimeError("formal-eskf did not initialize cleanly")
            if (self.build / "bin/px4-ekf2").exists():
                raise RuntimeError("POC build unexpectedly contains EKF2")
            self.setpoint = (0., 0., -5., 0.)
            self.pump(2.)
            self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1., 6., 0.])
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1., 0.])
            self.wait_for(self.armed, 5, "Normally armed (no force flag)")
            self.wait_for(lambda: ((self.messages["HEARTBEAT"][1].custom_mode >> 16) & 0xff) == 6,
                          5, "OFFBOARD active")
            POINTS = [("takeoff_hover", (0., 0., -5., 0.), 18.),
                      ("north", (8., 0., -5., 0.), 12.),
                      ("east_yaw", (8., 8., -5., math.pi / 4), 12.),
                      ("south", (0., 8., -5., math.pi / 4), 12.),
                      ("return_hover", (0., 0., -5., 0.), 12.)]
            for NAME, POINT, DURATION in POINTS:
                RESULT["phases"].append(self.phase(NAME, POINT, DURATION))
            self.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [0., 0., 0., math.nan, math.nan, math.nan, math.nan])
            self.setpoint = None
            self.wait_for(lambda: not self.armed(), 45, "Auto-land completed and automatically disarmed")
            self.pump(2.)
            RESULT["final_status"] = self.cli("formal_eskf", "status")
            if "fault=0" not in RESULT["final_status"]["output"] or "numerical_errors=0" not in RESULT["final_status"]["output"]:
                raise RuntimeError("Estimator reported a numerical failure")
            if self.args.check_faults:
                RESULT["ground_fault_checks"] = self.ground_fault_checks()
            RESULT["result"] = "flight_pass_pending_ulog_analysis"
        except Exception as ERROR:
            FAILURE = ERROR
            RESULT["error"] = str(ERROR)
            self.event(f"FAIL: {ERROR}")
            if self.process and self.process.poll() is None:
                RESULT["failure_status"] = [self.cli("formal_eskf", "status"), self.cli("commander", "status"),
                                            self.cli("simulator_sih", "status"), self.cli("logger", "status"),
                                            self.cli("listener", "failsafe_flags", "-n", "1")]
                if self.armed():
                    try:
                        self.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [0., 0., 0., math.nan, math.nan, math.nan, math.nan])
                        self.setpoint = None
                        self.wait_for(lambda: not self.armed(), 35, "Failure cleanup landed")
                    except Exception as CLEANUP_ERROR:
                        RESULT["cleanup_error"] = str(CLEANUP_ERROR)
        finally:
            if self.process and self.process.poll() is None:
                RESULT["shutdown"] = self.cli("shutdown")
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=5)
            CONSOLE.close()
            self.conn.close()
            RESULT["events"] = self.events
            RESULT["wall_duration_s"] = time.monotonic() - self.started
            RESULT["ulogs"] = [str(P.relative_to(self.output)) for P in self.rootfs.glob("log/**/*.ulg")]
            (self.output / "flight.json").write_text(json.dumps(RESULT, indent=2, allow_nan=False))
            (self.output / "telemetry.json").write_text(json.dumps(self.samples, allow_nan=False))
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
