#!/usr/bin/env python3
"""Use an isolated Kconfiglib checkout to configure CMake builds and PX4 packages."""

import argparse
from decimal import Decimal
import importlib
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True

REPO_DIR = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_DIR / "tools/kconfiglib"
TOOL_URL = "https://github.com/sysprog21/Kconfiglib.git"
TOOL_REVISION = "6d9102abd55b614d3c6133529ffa40dbea7fd2cf"


def load_kconfiglib():
    if not TOOL_DIR.exists():
        TOOL_DIR.parent.mkdir(parents=True, exist_ok=True)
        print(f"Fetching Kconfiglib {TOOL_REVISION}", file=sys.stderr, flush=True)
        with tempfile.TemporaryDirectory(prefix="kconfiglib-", dir=TOOL_DIR.parent) as TEMP:
            CHECKOUT = Path(TEMP) / "checkout"
            subprocess.run(["git", "clone", "--quiet", TOOL_URL, str(CHECKOUT)], check=True)
            subprocess.run(["git", "-C", str(CHECKOUT), "checkout", "--quiet", "--detach", TOOL_REVISION], check=True)
            CHECKOUT.rename(TOOL_DIR)

    REVISION = subprocess.check_output(["git", "-C", str(TOOL_DIR), "rev-parse", "HEAD"], text=True).strip()
    if REVISION != TOOL_REVISION:
        raise ValueError(f"{TOOL_DIR} must be at {TOOL_REVISION}, found {REVISION}")
    subprocess.run(["git", "-C", str(TOOL_DIR), "diff", "--exit-code", "HEAD", "--"], check=True)
    sys.path.insert(0, str(TOOL_DIR))
    return importlib.import_module("kconfiglib")


def load_config(KCONFIGLIB, CONFIG_FILE):
    CONFIG = KCONFIGLIB.Kconfig(str(REPO_DIR / "Kconfig"), warn_to_stderr=False)
    CONFIG.warn_assign_undef = True
    if CONFIG_FILE.exists():
        CONFIG.load_config(str(CONFIG_FILE))
    return CONFIG


def cmake_settings(CONFIG, KCONFIGLIB):
    """Reject ignored requests instead of silently building another profile."""
    ERRORS = list(CONFIG.warnings)
    SETTINGS = {}
    for SYMBOL in CONFIG.unique_defined_syms:
        VALUE = SYMBOL.str_value
        REQUESTED = SYMBOL.user_value
        if SYMBOL.type == KCONFIGLIB.BOOL:
            REQUESTED = {0: "n", 2: "y", None: None}[REQUESTED]
        if REQUESTED is not None and REQUESTED != VALUE:
            ERRORS.append(f"CONFIG_{SYMBOL.name} requested {REQUESTED!r}, resolves to {VALUE!r}; use menuconfig")
        if SYMBOL.name in ("ESKF_QUAT_EXACT", "ESKF_RESET_EXACT") or SYMBOL.name.startswith(("REPLAY_", "PX4_")):
            continue
        if SYMBOL.type == KCONFIGLIB.BOOL:
            SETTINGS[SYMBOL.name] = ("BOOL", "ON" if VALUE == "y" else "OFF")
        else:
            SETTINGS[SYMBOL.name] = ("STRING", VALUE)
    if ERRORS:
        raise ValueError("\n".join(ERRORS))
    BACKEND = "px4_matrix" if CONFIG.syms["REPLAY_PX4_MATRIX"].str_value == "y" else "eigen"
    SETTINGS["FORMAL_ESKF_REPLAY_BACKEND"] = ("STRING", BACKEND)
    return SETTINGS


def px4_header(CONFIG, KCONFIGLIB):
    SETTINGS = cmake_settings(CONFIG, KCONFIGLIB)
    LINES = ["// SPDX-License-Identifier: BSD-3-Clause\n#pragma once\n\n",
             "// Generated from Kconfig; configure through make menuconfig.\n"]
    for NAME in ("ESKF_CODEGEN", "ESKF_QUAT_APPROX", "ESKF_RESET_APPROX", "ESKF_MAG_TILT"):
        LINES.append(f"#define ESKF_CFG_{NAME[5:]} {int(SETTINGS[NAME][1] == 'ON')}\n")
    VALUES = {}
    ALLOW_ZERO = {"ACCEL_VARIANCE", "GYRO_VARIANCE", "ACCEL_BIAS_DENSITY", "GYRO_BIAS_DENSITY",
                  "FAULT_ACCEL_STD", "ACCEL_BIAS_LIMIT", "GYRO_BIAS_LIMIT", "MAG_STRENGTH_TOL",
                  "MAG_INCLINATION_TOL", "MAG_ACCEL_LIMIT", "GROUND_DEADZONE"}
    for SYMBOL in CONFIG.unique_defined_syms:
        if not SYMBOL.name.startswith("PX4_"):
            continue
        NAME = SYMBOL.name[4:]
        VALUE = SYMBOL.str_value
        if SYMBOL.type == KCONFIGLIB.BOOL:
            VALUE = str(int(VALUE == "y"))
        elif SYMBOL.type == KCONFIGLIB.INT:
            VALUE = str(int(VALUE, 10))
        elif SYMBOL.type == KCONFIGLIB.STRING:
            if not re.fullmatch(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?", VALUE):
                raise ValueError(f"{SYMBOL.name}: expected a decimal number")
            NUMBER = float(VALUE)
            if (not math.isfinite(NUMBER) or NUMBER < 0 or
                    (NUMBER == 0 and NAME not in ALLOW_ZERO) or
                    (NUMBER == 0 and not Decimal(VALUE).is_zero()) or
                    (NUMBER != 0 and not sys.float_info.min <= NUMBER * NUMBER <= sys.float_info.max)):
                raise ValueError(f"{SYMBOL.name}: value or its square is outside the supported numerical domain")
            if NAME in ("MAG_INCLINATION_TOL", "MAG_HEADING_NOISE") and NUMBER > math.pi:
                raise ValueError(f"{SYMBOL.name}: angle must not exceed pi radians")
            VALUE = repr(NUMBER)
        VALUES[NAME] = float(VALUE)
        LINES.append(f"#define ESKF_CFG_{NAME} {VALUE}\n")
    if VALUES["GNSS_MIN_POS_NOISE"] > VALUES["GNSS_MAX_POS_NOISE"]:
        raise ValueError("PX4_GNSS_MIN_POS_NOISE must not exceed PX4_GNSS_MAX_POS_NOISE")
    if not math.isfinite(200 * VALUES["GNSS_MAX_HDRIFT"] ** 2):
        raise ValueError("PX4_GNSS_MAX_HDRIFT overflows the GNSS motion clipping calculation")
    if VALUES["GNSS_MIN_HEIGHT_NOISE"] ** 2 > VALUES["GNSS_MAX_HEIGHT_VARIANCE"]:
        raise ValueError("PX4_GNSS_MIN_HEIGHT_NOISE squared must not exceed PX4_GNSS_MAX_HEIGHT_VARIANCE")
    return "".join(LINES)


def configure(CONFIG, KCONFIGLIB, BUILD_DIR, CMAKE_ARGS):
    SETTINGS = cmake_settings(CONFIG, KCONFIGLIB)
    # Non-menu options (compiler, toolchain, package paths, build type) stay in CMake.
    for INDEX, ARG in enumerate(CMAKE_ARGS):
        DEFINITION = CMAKE_ARGS[INDEX + 1] if ARG == "-D" and INDEX + 1 < len(CMAKE_ARGS) else ARG[2:]
        if ARG.startswith("-D") and DEFINITION.split("=", 1)[0].split(":", 1)[0] in SETTINGS:
            raise ValueError(f"{DEFINITION}: set menu-managed options in .config, not CMAKE_ARGS")

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    CACHE_FILE = BUILD_DIR / "kconfig.cmake"
    LINES = [f"# Generated from Kconfig by Kconfiglib {TOOL_REVISION}; do not edit.\n"]
    for NAME, (TYPE, VALUE) in SETTINGS.items():
        # CMake bracket arguments preserve paths, semicolons and literal ${...}.
        DELIMITER = "="
        while f"]{DELIMITER}]" in VALUE:
            DELIMITER += "="
        LINES.append(f'set({NAME} [{DELIMITER}[{VALUE}]{DELIMITER}] CACHE {TYPE} "Kconfig selection" FORCE)\n')
    CACHE_FILE.write_text("".join(LINES), encoding="utf-8")
    subprocess.run(["cmake", "-S", str(REPO_DIR), "-B", str(BUILD_DIR), *CMAKE_ARGS, "-C", str(CACHE_FILE)], check=True)


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("command", choices=("menuconfig", "defconfig", "configure", "px4-header"))
    PARSER.add_argument("--config", type=Path, default=REPO_DIR / ".config")
    PARSER.add_argument("--build-dir", type=Path, default=REPO_DIR / "build/demo")
    # CMake arguments follow --, so they cannot be mistaken for frontend options.
    ARGUMENTS = sys.argv[1:]
    SEPARATOR = ARGUMENTS.index("--") if "--" in ARGUMENTS else len(ARGUMENTS)
    ARGS = PARSER.parse_args(ARGUMENTS[:SEPARATOR])
    CMAKE_ARGS = ARGUMENTS[SEPARATOR + 1:]
    if CMAKE_ARGS and ARGS.command != "configure":
        PARSER.error("CMake arguments are only supported by configure")

    KCONFIGLIB = load_kconfiglib()
    CONFIG_FILE = ARGS.config.resolve()
    os.environ["KCONFIG_CONFIG"] = str(CONFIG_FILE)
    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    if ARGS.command == "defconfig":
        CONFIG = KCONFIGLIB.Kconfig(str(REPO_DIR / "Kconfig"))
        print(CONFIG.write_config(str(CONFIG_FILE)))
        return

    CONFIG = load_config(KCONFIGLIB, CONFIG_FILE)
    if ARGS.command == "px4-header":
        if not CONFIG_FILE.is_file():
            raise ValueError(f"configuration not found: {CONFIG_FILE}; run make menuconfig first")
        sys.stdout.write(px4_header(CONFIG, KCONFIGLIB))
        return
    if ARGS.command == "menuconfig":
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            raise ValueError("menuconfig needs an interactive terminal; use make defconfig for defaults")
        importlib.import_module("menuconfig").menuconfig(CONFIG)
        return

    if not CONFIG_FILE.exists():
        print(CONFIG.write_config(str(CONFIG_FILE)))
    configure(CONFIG, KCONFIGLIB, ARGS.build_dir.resolve(), CMAKE_ARGS)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as ERROR:
        sys.exit(f"error: {ERROR}")
