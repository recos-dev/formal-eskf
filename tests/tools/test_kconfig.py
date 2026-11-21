#!/usr/bin/env python3
"""Check menu selections against real CMake configuration and compiler flags."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True

REPO_DIR = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("configure", REPO_DIR / "scripts/configure.py")
FRONTEND = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FRONTEND)


class KconfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(CLS):
        CLS.KCONFIGLIB = FRONTEND.load_kconfiglib()

    def setUp(SELF):
        SELF.TEMP = tempfile.TemporaryDirectory(prefix="formal-eskf-config-")
        SELF.addCleanup(SELF.TEMP.cleanup)
        SELF.ROOT = Path(SELF.TEMP.name)
        SELF.CONFIG_FILE = SELF.ROOT / ".config"
        SELF.BUILD_DIR = SELF.ROOT / "build"

    def read_config(SELF, TEXT=""):
        SELF.CONFIG_FILE.write_text(TEXT, encoding="utf-8")
        return FRONTEND.load_config(SELF.KCONFIGLIB, SELF.CONFIG_FILE)

    def run_configure(SELF, *ARGS):
        RESULT = subprocess.run(
            [sys.executable, str(REPO_DIR / "scripts/configure.py"), "configure",
             "--config", str(SELF.CONFIG_FILE), "--build-dir", str(SELF.BUILD_DIR), "--", *ARGS],
            text=True, capture_output=True,
        )
        SELF.assertEqual(RESULT.returncode, 0, RESULT.stdout + RESULT.stderr)

    def cache(SELF, BUILD_DIR):
        return dict(LINE.split("=", 1) for LINE in (BUILD_DIR / "CMakeCache.txt").read_text().splitlines()
                    if "=" in LINE and not LINE.startswith(("//", "#")))

    def test_defaults_match_direct_cmake(SELF):
        SELF.run_configure()
        DIRECT_DIR = SELF.ROOT / "direct"
        subprocess.run(["cmake", "-S", str(REPO_DIR), "-B", str(DIRECT_DIR)],
                       check=True, stdout=subprocess.DEVNULL)
        CONFIG = FRONTEND.load_config(SELF.KCONFIGLIB, SELF.CONFIG_FILE)
        MENU_CACHE, DIRECT_CACHE = SELF.cache(SELF.BUILD_DIR), SELF.cache(DIRECT_DIR)
        for NAME, (TYPE, _) in FRONTEND.cmake_settings(CONFIG, SELF.KCONFIGLIB).items():
            SELF.assertEqual(MENU_CACHE[f"{NAME}:{TYPE}"], DIRECT_CACHE[f"{NAME}:{TYPE}"], NAME)

    def test_mode_changes_replace_cache_and_compiler_flags(SELF):
        for APPROX, TILT, BOOL, BIT, TILT_BIT in (("y", "n", "ON", "1", "0"), ("n", "y", "OFF", "0", "1")):
            PREDICTION = "APPROX" if APPROX == "y" else "EXACT"
            SELF.read_config(f"CONFIG_ESKF_QUAT_{PREDICTION}=y\n"
                             f"CONFIG_ESKF_RESET_{PREDICTION}=y\nCONFIG_ESKF_MAG_TILT={TILT}\n"
                             f"CONFIG_ESKF_CODEGEN={TILT}\n")
            SELF.run_configure("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
            CACHE = SELF.cache(SELF.BUILD_DIR)
            SELF.assertEqual(CACHE["ESKF_QUAT_APPROX:BOOL"], BOOL)
            SELF.assertEqual(CACHE["ESKF_RESET_APPROX:BOOL"], BOOL)
            COMMANDS = json.loads((SELF.BUILD_DIR / "compile_commands.json").read_text())
            CORE_COMMANDS = [ENTRY["command"] for ENTRY in COMMANDS if "ESKF_QUAT_APPROX" in ENTRY["command"]]
            SELF.assertTrue(CORE_COMMANDS)
            for COMMAND in CORE_COMMANDS:
                SELF.assertIn(f"-DESKF_QUAT_APPROX={BIT}", COMMAND)
                SELF.assertIn(f"-DESKF_RESET_APPROX={BIT}", COMMAND)
                SELF.assertIn(f"-DESKF_MAG_TILT={TILT_BIT}", COMMAND)
                SELF.assertIn(f"-DESKF_CODEGEN={TILT_BIT}", COMMAND)

    def test_invalid_requests_are_rejected(SELF):
        for TEXT in (
            "CONFIG_UNKNOWN=y\n",
            "CONFIG_ESKF_QUAT_APPROX=maybe\n",
            "CONFIG_ESKF_QUAT_APPROX=y\nCONFIG_ESKF_QUAT_APPROX=n\n",
            "CONFIG_ESKF_QUAT_EXACT=y\nCONFIG_ESKF_QUAT_APPROX=y\n",
            "CONFIG_ESKF_RESET_EXACT=y\nCONFIG_ESKF_RESET_APPROX=y\n",
            "CONFIG_FORMAL_ESKF_BUILD_EIGEN_BACKEND=n\nCONFIG_FORMAL_ESKF_BUILD_EXAMPLES=y\n",
            "CONFIG_FORMAL_ESKF_BUILD_EXAMPLES=y\nCONFIG_REPLAY_PX4_MATRIX=y\n",
        ):
            with SELF.subTest(TEXT=TEXT), SELF.assertRaises(ValueError):
                FRONTEND.cmake_settings(SELF.read_config(TEXT), SELF.KCONFIGLIB)

    def test_px4_replay_choice_and_literal_include_paths(SELF):
        INCLUDES = str(SELF.ROOT / "PX4 headers/${literal}") + ";" + str(SELF.ROOT / "generated]=]")
        SELF.read_config(
            "CONFIG_FORMAL_ESKF_BUILD_EIGEN_BACKEND=n\n"
            "CONFIG_FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND=y\n"
            f'CONFIG_FORMAL_ESKF_PX4_INCLUDE_DIRS="{INCLUDES}"\n'
            "CONFIG_FORMAL_ESKF_BUILD_EXAMPLES=y\nCONFIG_REPLAY_PX4_MATRIX=y\n"
        )
        SELF.run_configure()
        CACHE = SELF.cache(SELF.BUILD_DIR)
        SELF.assertEqual(CACHE["FORMAL_ESKF_REPLAY_BACKEND:STRING"], "px4_matrix")
        SELF.assertEqual(CACHE["FORMAL_ESKF_PX4_INCLUDE_DIRS:STRING"], INCLUDES)

    def test_conflicting_cmake_arguments_are_rejected(SELF):
        CONFIG = SELF.read_config()
        for ARGS in (("-DESKF_QUAT_APPROX=ON",), ("-D", "ESKF_MAG_TILT:BOOL=OFF")):
            with SELF.subTest(ARGS=ARGS), SELF.assertRaisesRegex(ValueError, "menu-managed"):
                FRONTEND.configure(CONFIG, SELF.KCONFIGLIB, SELF.BUILD_DIR, ARGS)

    def test_px4_defaults_and_consumers(SELF):
        CONFIG = SELF.read_config()
        HEADER = FRONTEND.px4_header(CONFIG, SELF.KCONFIGLIB)
        MODULE = REPO_DIR / "integrations/px4/module"
        SELF.assertEqual(HEADER, (MODULE / "DeploymentConfig.hpp").read_text())
        SOURCES = "\n".join(FILE.read_text() for FILE in MODULE.glob("*")
                            if FILE.suffix in (".cpp", ".hpp") and FILE.name != "DeploymentConfig.hpp")
        for SYMBOL in CONFIG.unique_defined_syms:
            if SYMBOL.name.startswith("PX4_"):
                SELF.assertIn("ESKF_CFG_" + SYMBOL.name[4:], SOURCES, SYMBOL.name)

    def test_px4_invalid_numbers_and_dependencies(SELF):
        for TEXT in (
            'CONFIG_PX4_MAG_GATE="nan"\n',
            'CONFIG_PX4_MAG_GATE="1e200"\n',
            'CONFIG_PX4_MAG_GATE="1e-200"\n',
            'CONFIG_PX4_ACCEL_VARIANCE="-1"\n',
            'CONFIG_PX4_ACCEL_VARIANCE="1e-1000"\n',
            'CONFIG_PX4_MAG_HEADING_NOISE="4"\n',
            'CONFIG_PX4_GNSS_MIN_POS_NOISE="11"\n',
            'CONFIG_PX4_GNSS_MAX_HDRIFT="1e153"\n',
            'CONFIG_PX4_GNSS_MIN_HEIGHT_NOISE="11"\n',
            'CONFIG_PX4_WMM=n\nCONFIG_PX4_MAG_REQUIRE_WMM=y\n',
            'CONFIG_PX4_GNSS_HEALTH_MS=0\n',
            'CONFIG_PX4_MAG_INTERVAL_MS=2147483648\n',
        ):
            with SELF.subTest(TEXT=TEXT), SELF.assertRaises(ValueError):
                FRONTEND.px4_header(SELF.read_config(TEXT), SELF.KCONFIGLIB)

    def test_px4_configured_package(SELF):
        CONFIG = SELF.read_config('CONFIG_PX4_WMM=n\nCONFIG_PX4_GROUND_EFFECT=n\n'
                                 'CONFIG_PX4_MAG_GATE="4.5"\nCONFIG_PX4_MAG_HEALTH_MS=0\n'
                                 'CONFIG_PX4_GNSS_TIMEOUT_MS=004000\nCONFIG_ESKF_RESET_APPROX=y\nCONFIG_ESKF_CODEGEN=n\n')
        PACKAGE = SELF.ROOT / "package"
        RESULT = subprocess.run([sys.executable, str(REPO_DIR / "integrations/px4/scripts/install.py"),
                                 "--export", str(PACKAGE), "--config", str(SELF.CONFIG_FILE)],
                                text=True, capture_output=True)
        SELF.assertEqual(RESULT.returncode, 0, RESULT.stdout + RESULT.stderr)
        MODULE = PACKAGE / "overlay/src/modules/formal_eskf"
        HEADER = (MODULE / "DeploymentConfig.hpp").read_text()
        SELF.assertEqual(HEADER, FRONTEND.px4_header(CONFIG, SELF.KCONFIGLIB))
        SELF.assertIn("#define ESKF_CFG_WMM 0\n", HEADER)
        SELF.assertIn("#define ESKF_CFG_GROUND_EFFECT 0\n", HEADER)
        SELF.assertIn("#define ESKF_CFG_MAG_GATE 4.5\n", HEADER)
        SELF.assertIn("#define ESKF_CFG_GNSS_TIMEOUT_MS 4000\n", HEADER)
        SELF.assertIn("#define ESKF_CFG_RESET_APPROX 1\n", HEADER)
        SELF.assertIn("#define ESKF_CFG_CODEGEN 0\n", HEADER)
        SELF.assertEqual((MODULE / "configuration.config").read_bytes(), SELF.CONFIG_FILE.read_bytes())
        MANIFEST = json.loads((PACKAGE / "manifest.json").read_text())
        SELF.assertIn("src/modules/formal_eskf/DeploymentConfig.hpp", MANIFEST["files"])
        SELF.assertIn("src/modules/formal_eskf/configuration.config", MANIFEST["files"])
        RELOCATED = SELF.ROOT / "relocated"
        subprocess.run([sys.executable, str(PACKAGE / "install.py"), "--export", str(RELOCATED)],
                       check=True, stdout=subprocess.DEVNULL)
        SELF.assertEqual(json.loads((RELOCATED / "manifest.json").read_text()), MANIFEST)


if __name__ == "__main__":
    unittest.main()
