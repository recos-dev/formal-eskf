#!/usr/bin/env python3
"""Offline negative controls for the planned-test runner."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

REPO_DIR = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("verify_tests", REPO_DIR / "scripts/verify_tests.py")
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
ID = "TC-C-FAILURE-STATUS"
SOURCE = "tests/common/test_contracts.cpp"


class TestPlan(unittest.TestCase):
    def setUp(SELF):
        SELF.TEMP = tempfile.TemporaryDirectory()
        SELF.addCleanup(SELF.TEMP.cleanup)
        SELF.ROOT = Path(SELF.TEMP.name)
        (SELF.ROOT / SOURCE).parent.mkdir(parents=True)
        (SELF.ROOT / SOURCE).write_text("// Test source fixture.\n")
        SELF.PLAN_DIR = SELF.ROOT / "docs"
        SELF.PLAN_DIR.mkdir()
        SELF.PLAN = {ID: {"kind": "Native", "sources": [SOURCE], "binding": "Native", "design": "docs/cases.md"}}
        SELF.CTEST = {"tests": [{"name": "fixture", "command": ["fixture-executable"],
                                "properties": [{"name": "LABELS", "value": ["plan-source:" + SOURCE]}]}]}

    def write_case(SELF, BINDING, CASE_ID=ID):
        return f"| {CASE_ID} | requirement | inputs | oracle | {BINDING} |\n"

    def test_plan_binding_and_duplicate_detection(SELF):
        FILE = SELF.PLAN_DIR / "cases.md"
        ROW = SELF.write_case("Native: [source](../" + SOURCE + ")")
        FILE.write_text(ROW)
        PLAN = RUNNER.read_plan(SELF.PLAN_DIR, SELF.ROOT)
        SELF.assertEqual(PLAN[ID]["sources"], [SOURCE])
        FILE.write_text(ROW + ROW)
        with SELF.assertRaisesRegex(ValueError, "duplicate"):
            RUNNER.read_plan(SELF.PLAN_DIR, SELF.ROOT)
        FILE.write_text(ROW.rstrip() + " extra |\n")
        with SELF.assertRaisesRegex(ValueError, "malformed"):
            RUNNER.read_plan(SELF.PLAN_DIR, SELF.ROOT)

    def test_missing_source_and_false_native_binding(SELF):
        FILE = SELF.PLAN_DIR / "cases.md"
        for BINDING in ("Native: no executable", "Partial: [source](../tests/absent.cpp)"):
            FILE.write_text(SELF.write_case(BINDING))
            with SELF.assertRaises(ValueError):
                RUNNER.read_plan(SELF.PLAN_DIR, SELF.ROOT)
        FILE.write_text("# No case rows\n")
        with SELF.assertRaisesRegex(ValueError, "empty"):
            RUNNER.read_plan(SELF.PLAN_DIR, SELF.ROOT)

    def test_registry_rejects_empty_duplicate_or_malformed_results(SELF):
        SELF.assertEqual(RUNNER.parse_registry(f"Backend: Eigen\nCompiler: GCC\n{ID} binary64\n"),
                         [(ID, "binary64")])
        for OUTPUT in ("Backend: Eigen\n", f"{ID} binary64\n{ID} binary64\n", "unparseable\n",
                       f"{ID} binary64 extra\n"):
            with SELF.assertRaises(ValueError):
                RUNNER.parse_registry(OUTPUT)

    def collect(SELF, OUTPUT, CODE=0):
        with patch.object(RUNNER, "run_command", return_value=(CODE, OUTPUT)):
            return RUNNER.collect_tests(SELF.CTEST, SELF.PLAN, SELF.ROOT, 1, SELF.ROOT)

    def test_registry_reconciles_id_and_owning_source(SELF):
        TESTS, ERRORS, NOT_SELECTED = SELF.collect(f"{ID} binary64\n")
        SELF.assertFalse(ERRORS)
        SELF.assertEqual(len(TESTS), 1)
        SELF.assertEqual(TESTS[0]["command"][-4:], ["--case", ID, "--profile", "binary64"])
        for OUTPUT, CODE in (("TC-UNKNOWN-case binary64\n", 0), (f"{ID} binary64\n", 1), ("", 0)):
            _, ERRORS, _ = SELF.collect(OUTPUT, CODE)
            SELF.assertTrue(ERRORS)
        SELF.PLAN["TC-SECOND-case"] = dict(SELF.PLAN[ID])
        _, ERRORS, _ = SELF.collect(f"{ID} binary64\n")
        SELF.assertTrue(any("TC-SECOND-case" in ERROR for ERROR in ERRORS))

    def test_shared_id_cannot_hide_one_sources_missing_case(SELF):
        SECOND = "tests/common/second.cpp"
        SELF.PLAN[ID]["sources"].append(SECOND)
        SELF.PLAN["TC-SECOND-case"] = {"kind": "Native", "sources": [SECOND], "binding": "Native"}
        SELF.CTEST["tests"].append({"name": "second", "command": ["second-executable"],
                                    "properties": [{"name": "LABELS", "value": ["plan-source:" + SECOND]}]})
        with patch.object(RUNNER, "run_command", side_effect=[(0, f"{ID} binary64\n"),
                                                                (0, "TC-SECOND-case binary64\n")]):
            _, ERRORS, _ = RUNNER.collect_tests(SELF.CTEST, SELF.PLAN, SELF.ROOT, 1, SELF.ROOT)
        SELF.assertTrue(any(f"second: missing source-bound IDs: {ID}" in ERROR for ERROR in ERRORS))

    def test_disabled_missing_and_unresolved_are_not_passes(SELF):
        SELF.CTEST["tests"][0]["properties"].append({"name": "DISABLED", "value": True})
        _, ERRORS, _ = SELF.collect(f"{ID} binary64\n")
        SELF.assertTrue(ERRORS)
        SELF.CTEST["tests"] = []
        _, ERRORS, _ = SELF.collect("")
        SELF.assertTrue(ERRORS)
        SELF.PLAN[ID] = {"kind": "Pending", "sources": [], "binding": "Pending"}
        _, ERRORS, _ = SELF.collect("")
        SELF.assertTrue(any("unresolved" in ERROR for ERROR in ERRORS))
        SELF.PLAN[ID] = {"kind": "Proved", "sources": [], "binding": "Proved: native run"}
        _, ERRORS, _ = SELF.collect("")
        SELF.assertTrue(any("unresolved" in ERROR for ERROR in ERRORS))

    def test_analysis_and_delegation_do_not_become_executable_passes(SELF):
        SELF.CTEST["tests"] = []
        for KIND in ("Analysis", "Delegated"):
            SELF.PLAN[ID] = {"kind": KIND, "sources": [], "binding": KIND + ": owner review"}
            TESTS, ERRORS, NOT_SELECTED = SELF.collect("")
            SELF.assertEqual(TESTS, [])
            SELF.assertEqual(ERRORS, [])

    def execute(SELF, TEXT, CODE=0, TIMEOUT=1):
        TEST = {"id": ID, "profile": "binary64", "name": "fixture", "directory": str(SELF.ROOT),
                "environment": None, "command": [sys.executable, "-c", f"print({TEXT!r});raise SystemExit({CODE})"]}
        return RUNNER.execute_case(TEST, TIMEOUT, SELF.ROOT / "case.log")

    def test_exit_status_and_exact_selected_case_are_both_required(SELF):
        GOOD = f"RUN {ID} binary64\nPASS {ID} binary64"
        SELF.assertTrue(SELF.execute(GOOD)["passed"])
        for TEXT, CODE in ((GOOD, 1), ("", 0), (f"PASS {ID} binary64", 0),
                           (GOOD + "\n" + GOOD, 0), (GOOD + "\nERROR: no assertions", 0),
                           (GOOD + "\nRUN TC-OTHER binary64\nPASS TC-OTHER binary64", 0),
                           (f"RUN {ID} binary64\nFAIL {ID} binary64", 0)):
            SELF.assertFalse(SELF.execute(TEXT, CODE)["passed"])

    def test_missing_program_and_timeout_fail(SELF):
        TEST = {"id": ID, "profile": "binary64", "name": "fixture", "directory": str(SELF.ROOT),
                "environment": None, "command": [str(SELF.ROOT / "absent")]}
        SELF.assertFalse(RUNNER.execute_case(TEST, 1, SELF.ROOT / "missing.log")["passed"])
        TEST["command"] = [sys.executable, "-c", "import time;time.sleep(5)"]
        RESULT = RUNNER.execute_case(TEST, 0.02, SELF.ROOT / "timeout.log")
        SELF.assertFalse(RESULT["passed"])
        SELF.assertEqual(RESULT["exit_code"], 124)

    def test_display_totals_and_exit_status(SELF):
        for PASS, EXPECTED in ((True, 0), (False, 1)):
            def COMMAND(COMMAND, *_ARGS, **_KWARGS):
                if COMMAND[0] == "ctest":
                    return 0, json.dumps(SELF.CTEST)
                if COMMAND[-1] == "--list":
                    return 0, f"{ID} binary64\n"
                return (0 if PASS else 1), f"RUN {ID} binary64\n{'PASS' if PASS else 'FAIL'} {ID} binary64\n"

            OUTPUT = io.StringIO()
            with patch.object(RUNNER, "REPO_DIR", SELF.ROOT), \
                    patch.object(RUNNER, "read_plan", return_value=SELF.PLAN), \
                    patch.object(RUNNER, "run_command", side_effect=COMMAND), \
                    patch.object(RUNNER, "write_report"), \
                    patch.object(sys, "argv", ["verify_tests.py", "--no-build", "--build-dir", str(SELF.ROOT)]), \
                    contextlib.redirect_stdout(OUTPUT):
                SELF.assertEqual(RUNNER.main(), EXPECTED)
            SELF.assertIn(f"[{'Pass' if PASS else 'Failed'}] {ID}", OUTPUT.getvalue())
            SELF.assertIn(f"{int(PASS)} passed, {int(not PASS)} failed, 0 registration gaps", OUTPUT.getvalue())


if __name__ == "__main__":
    unittest.main()
