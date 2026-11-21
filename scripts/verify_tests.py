#!/usr/bin/env python3
"""Build and run the requirement-bound core/runtime tests, one case at a time."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

REPO_DIR = Path(__file__).resolve().parents[1]
PLAN_DIR = REPO_DIR / "docs/spec/verification/tests"
CASE_PATTERN = re.compile(r"TC-[A-Za-z0-9_.-]+")
PASS_COLOR = "\033[32;01m"
FAIL_COLOR = "\033[31;01m"
NO_COLOR = "\033[0m"


def read_plan(DIRECTORY, ROOT):
    CASES = {}
    for FILE in sorted(DIRECTORY.rglob("*.md")):
        for LINE in FILE.read_text().splitlines():
            CELLS = [CELL.strip() for CELL in LINE.split("|")]
            if len(CELLS) < 2 or not CASE_PATTERN.fullmatch(CELLS[1]):
                continue
            if len(CELLS) != 8:
                raise ValueError(f"malformed planned case row in {FILE}: {CELLS[1]}")
            ID, NAME, BINDING = CELLS[1], CELLS[2], CELLS[6]
            if not NAME:
                raise ValueError(f"missing case name in {FILE}: {ID}")
            if ID in CASES:
                raise ValueError(f"duplicate planned ID: {ID}")
            KIND = BINDING.split(":", 1)[0]
            SOURCES = []
            for LINK in re.findall(r"\]\(([^)#]+)(?:#[^)]*)?\)", BINDING):
                SOURCE = (FILE.parent / LINK).resolve()
                if SOURCE.is_relative_to(ROOT / "tests"):
                    if not SOURCE.is_file():
                        raise ValueError(f"{ID}: missing bound source {SOURCE}")
                    SOURCES.append(SOURCE.relative_to(ROOT).as_posix())
            if KIND in ("Native", "Partial") and not SOURCES:
                raise ValueError(f"{ID}: {KIND} binding has no test source")
            CASES[ID] = {"name": NAME, "kind": KIND, "sources": SOURCES, "binding": BINDING,
                         "design": FILE.relative_to(ROOT).as_posix()}
    if not CASES:
        raise ValueError("empty test design")
    return CASES


def run_command(COMMAND, DIRECTORY, TIMEOUT, ENVIRONMENT=None):
    try:
        with subprocess.Popen(COMMAND, cwd=DIRECTORY, env=ENVIRONMENT, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              start_new_session=os.name == "posix") as PROCESS:
            try:
                OUTPUT, _ = PROCESS.communicate(timeout=TIMEOUT)
                return PROCESS.returncode, OUTPUT
            except subprocess.TimeoutExpired:
                # Stop owned compiler children too; a timed-out Python case may
                # otherwise leave a build consuming memory after a failed result.
                if os.name == "posix":
                    os.killpg(PROCESS.pid, signal.SIGKILL)
                else:
                    PROCESS.kill()
                OUTPUT, _ = PROCESS.communicate()
                return 124, OUTPUT + f"\nERROR: timed out after {TIMEOUT} seconds\n"
    except OSError as ERROR:
        return 127, str(ERROR) + "\n"


def parse_registry(OUTPUT):
    ENTRIES = []
    for LINE in OUTPUT.splitlines():
        if not LINE.strip() or LINE.startswith(("Backend: ", "Compiler: ", "ESKF_")):
            continue
        FIELDS = LINE.split()
        if len(FIELDS) != 2 or not CASE_PATTERN.fullmatch(FIELDS[0]):
            raise ValueError(f"invalid case registry line: {LINE}")
        ENTRY = tuple(FIELDS)
        if ENTRY in ENTRIES:
            raise ValueError(f"duplicate case/profile registration: {LINE}")
        ENTRIES.append(ENTRY)
    if not ENTRIES:
        raise ValueError("empty executable case registry")
    return ENTRIES


def collect_tests(CTEST, PLAN, BUILD_DIR, TIMEOUT, LOG_DIR):
    TESTS, ERRORS, NOT_SELECTED = [], [], []
    BOUND_SOURCES = {SOURCE for CASE in PLAN.values() for SOURCE in CASE["sources"]}
    CONFIGURED_SOURCES = set()
    for TEST in CTEST.get("tests", []):
        PROPERTIES = {ITEM["name"]: ITEM["value"] for ITEM in TEST.get("properties", [])}
        SOURCES = [LABEL.removeprefix("plan-source:") for LABEL in PROPERTIES.get("LABELS", [])
                   if LABEL.startswith("plan-source:")]
        CONFIGURED_SOURCES.update(SOURCES)
        if not any(SOURCE in BOUND_SOURCES for SOURCE in SOURCES):
            continue
        NAME = TEST["name"]
        if len(SOURCES) != 1 or not TEST.get("command") or PROPERTIES.get("DISABLED"):
            ERRORS.append(f"{NAME}: requires one source label and an enabled executable command")
            continue
        DIRECTORY = Path(PROPERTIES.get("WORKING_DIRECTORY", BUILD_DIR))
        ENVIRONMENT = dict(os.environ)
        for SETTING in PROPERTIES.get("ENVIRONMENT", []):
            KEY, VALUE = SETTING.split("=", 1)
            ENVIRONMENT[KEY] = VALUE
        CODE, OUTPUT = run_command(TEST["command"] + ["--list"], DIRECTORY, TIMEOUT, ENVIRONMENT)
        (LOG_DIR / f"{NAME}.list.log").write_text(OUTPUT)
        try:
            if CODE:
                raise ValueError(f"registry command exited {CODE}")
            ENTRIES = parse_registry(OUTPUT)
            EXPECTED = {ID for ID, CASE in PLAN.items() if SOURCES[0] in CASE["sources"]}
            MISSING = EXPECTED - {ID for ID, _ in ENTRIES}
            if MISSING:
                ERRORS.append(f"{NAME}: missing source-bound IDs: {', '.join(sorted(MISSING))}")
            for ID, PROFILE in ENTRIES:
                if ID not in PLAN or SOURCES[0] not in PLAN[ID]["sources"]:
                    raise ValueError(f"unbound case {ID} in {SOURCES[0]}")
                TESTS.append({"name": NAME, "id": ID, "profile": PROFILE, "source": SOURCES[0],
                              "command": TEST["command"] + ["--case", ID, "--profile", PROFILE],
                              "directory": str(DIRECTORY), "environment": ENVIRONMENT})
        except ValueError as ERROR:
            ERRORS.append(f"{NAME}: {ERROR}")
    CACHE_FILE = BUILD_DIR / "CMakeCache.txt"
    CACHE = CACHE_FILE.read_text() if CACHE_FILE.is_file() else ""
    CMSIS_DISABLED = re.search(r"^FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND:[^=\n]+=OFF$", CACHE, re.MULTILINE) is not None
    for SOURCE in sorted(BOUND_SOURCES - CONFIGURED_SOURCES):
        if SOURCE == "tests/math/test_cmsis_dsp_backend.cpp" and CMSIS_DISABLED:
            NOT_SELECTED.append({"source": SOURCE, "reason": "FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND=OFF"})
        else:
            ERRORS.append(f"{SOURCE}: no source-labeled target in the configured build")
    FOUND = {TEST["id"] for TEST in TESTS}
    for ID, CASE in PLAN.items():
        if CASE["kind"] in ("Native", "Partial") and ID not in FOUND:
            ERRORS.append(f"{ID}: no executable registration in the configured build")
        elif CASE["kind"] not in ("Native", "Partial", "Analysis", "Delegated"):
            ERRORS.append(f"{ID}: unresolved binding: {CASE['binding']}")
    return TESTS, ERRORS, NOT_SELECTED


def execute_case(TEST, TIMEOUT, LOG_FILE):
    CODE, OUTPUT = run_command(TEST["command"], TEST["directory"], TIMEOUT, TEST["environment"])
    LOG_FILE.write_text(OUTPUT)
    SUFFIX = TEST["id"] + " " + TEST["profile"]
    LINES = OUTPUT.splitlines()
    # A successful process without the selected case's assertion-bearing result
    # is not a pass. The C++ runner independently rejects zero assertions.
    MARKERS = [LINE for LINE in LINES if LINE.startswith(("RUN ", "PASS ", "FAIL "))]
    PASSED = (CODE == 0 and MARKERS == ["RUN " + SUFFIX, "PASS " + SUFFIX] and
              not any(LINE.startswith("ERROR:") for LINE in LINES))
    return {"id": TEST["id"], "profile": TEST["profile"], "target": TEST["name"],
            "passed": PASSED, "exit_code": CODE, "log": str(LOG_FILE)}


def write_report(LOG_DIR, PLAN, RESULTS, ERRORS, ELAPSED, BUILD_DIR, TESTS, BUILD_PERFORMED, NOT_SELECTED):
    CODE, HEAD = run_command(["git", "rev-parse", "HEAD"], REPO_DIR, 30)
    _, STATUS = run_command(["git", "status", "--short"], REPO_DIR, 30)
    FILES = set(PLAN_DIR.rglob("*.md"))
    FILES.update(REPO_DIR / SOURCE for CASE in PLAN.values() for SOURCE in CASE["sources"])
    FILES.update((REPO_DIR / "include/formal_eskf").rglob("*.hpp"))
    FILES.update(PATH for PATH in (REPO_DIR / "tests/support").rglob("*") if PATH.is_file())
    FILES.add(Path(__file__).resolve())
    FILES.update(PATH for PATH in (BUILD_DIR / "CMakeCache.txt", BUILD_DIR / "compile_commands.json")
                 if PATH.is_file())
    FILES.update(Path(TEST["command"][0]) for TEST in TESTS if Path(TEST["command"][0]).is_file())
    HASHES = {str(PATH): hashlib.sha256(PATH.read_bytes()).hexdigest() for PATH in sorted(FILES)}
    REPORT = {"scope": "Configured native core/runtime cases; not proof or target qualification",
              "head": HEAD.strip() if CODE == 0 else None, "working_tree": STATUS,
              "build_dir": str(BUILD_DIR), "build_performed": BUILD_PERFORMED,
              "freshness": "Build executed" if BUILD_PERFORMED else "Existing binaries; source freshness not checked",
              "file_sha256": HASHES, "plan": PLAN,
              "not_selected_sources": NOT_SELECTED,
              "results": RESULTS, "errors": ERRORS, "elapsed_seconds": ELAPSED}
    (LOG_DIR / "report.json").write_text(json.dumps(REPORT, indent=2) + "\n")


def case_label(TEST, PLAN):
    BACKENDS = {"eigen": "Eigen", "px4_matrix": "PX4 matrix", "cmsis_dsp": "CMSIS-DSP", "standard": "StandardMath"}
    BACKEND = BACKENDS.get(TEST["name"].rsplit(".", 1)[-1])
    PROFILE = f"{BACKEND}, {TEST['profile']}" if BACKEND else TEST["profile"]
    return f"{PLAN[TEST['id']]['name']} ({PROFILE})"


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--build-dir", type=Path, default=REPO_DIR / "build/tests")
    PARSER.add_argument("--no-build", action="store_true", help="use an already configured and built profile")
    PARSER.add_argument("--jobs", type=int, default=2)
    PARSER.add_argument("--timeout", type=float, default=180, help="seconds per case or registry command")
    PARSER.add_argument("--case", action="append", default=[], help="run only these IDs after full registry checks")
    PARSER.add_argument("--list", action="store_true", help="list reconciled case/profile registrations")
    ARGS = PARSER.parse_args()
    if ARGS.jobs < 1 or not math.isfinite(ARGS.timeout) or ARGS.timeout <= 0:
        PARSER.error("jobs and timeout must be positive")
    START = time.monotonic()
    BUILD_DIR = ARGS.build_dir.resolve()
    OUTPUT_DIR = REPO_DIR / "output/tests"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR = Path(tempfile.mkdtemp(prefix=time.strftime("%Y%m%d-%H%M%S-"), dir=OUTPUT_DIR))
    try:
        PLAN = read_plan(PLAN_DIR, REPO_DIR)
        UNKNOWN = set(ARGS.case) - PLAN.keys()
        if UNKNOWN:
            raise ValueError("unknown requested IDs: " + ", ".join(sorted(UNKNOWN)))
        if not ARGS.no_build:
            COMMANDS = [(["cmake", "-S", str(REPO_DIR), "-B", str(BUILD_DIR), "-DBUILD_TESTING=ON",
                          "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"], "configure"),
                        (["cmake", "--build", str(BUILD_DIR), "--parallel", str(ARGS.jobs)], "build")]
            for COMMAND, NAME in COMMANDS:
                print(f"{NAME.capitalize()} tests...", flush=True)
                CODE, OUTPUT = run_command(COMMAND, REPO_DIR, 1800)
                (LOG_DIR / f"{NAME}.log").write_text(OUTPUT)
                if CODE:
                    raise ValueError(f"{NAME} failed ({CODE}); see {LOG_DIR / (NAME + '.log')}")
        CODE, OUTPUT = run_command(["ctest", "--test-dir", str(BUILD_DIR), "--show-only=json-v1"], REPO_DIR, 30)
        if CODE:
            raise ValueError(f"CTest inventory failed ({CODE}): {OUTPUT}")
        (LOG_DIR / "ctest.json").write_text(OUTPUT)
        TESTS, ERRORS, NOT_SELECTED = collect_tests(json.loads(OUTPUT), PLAN, BUILD_DIR, ARGS.timeout, LOG_DIR)
        for ERROR in ERRORS:
            print(f"[Missing] {ERROR}")
        for ITEM in NOT_SELECTED:
            print(f"[Not selected] {ITEM['source']}: {ITEM['reason']}")
        RESULTS = []
        for INDEX, TEST in enumerate(TESTS):
            if ARGS.case and TEST["id"] not in ARGS.case:
                continue
            LABEL = case_label(TEST, PLAN)
            if ARGS.list:
                print(f"{TEST['id']}: {LABEL}")
                continue
            RESULT = execute_case(TEST, ARGS.timeout, LOG_DIR / f"case-{INDEX:04d}.log")
            RESULTS.append(RESULT)
            COLOR = PASS_COLOR if RESULT["passed"] else FAIL_COLOR
            print(f"{COLOR}[{'Pass' if RESULT['passed'] else 'Fail'}]{NO_COLOR} {LABEL}", flush=True)
            if not RESULT["passed"]:
                print(f"  Log: {RESULT['log']}")
        PASS_COUNT = sum(RESULT["passed"] for RESULT in RESULTS)
        FAIL_COUNT = len(RESULTS) - PASS_COUNT
        PARTIAL = sum(CASE["kind"] == "Partial" for CASE in PLAN.values())
        NON_EXECUTABLE = sum(CASE["kind"] in ("Analysis", "Delegated") for CASE in PLAN.values())
        ELAPSED = time.monotonic() - START
        GAP_COLOR = FAIL_COLOR if ERRORS else PASS_COLOR
        print(f"\nExecution: {PASS_COLOR}{PASS_COUNT} passed{NO_COLOR}, "
              f"{FAIL_COLOR}{FAIL_COUNT} failed{NO_COLOR}, "
              f"{GAP_COLOR}{len(ERRORS)} registration gaps{NO_COLOR}")
        print(f"Design: {len(PLAN)} IDs; {PARTIAL} partial acceptance, {NON_EXECUTABLE} analysis/delegated obligations")
        print(f"Total elapsed: {ELAPSED:.1f} s\nReport: {LOG_DIR / 'report.json'}")
        write_report(LOG_DIR, PLAN, RESULTS, ERRORS, ELAPSED, BUILD_DIR, TESTS, not ARGS.no_build, NOT_SELECTED)
        return 2 if ERRORS else int(FAIL_COUNT != 0)
    except (ValueError, OSError, KeyError) as ERROR:
        print(f"error: {ERROR}\nLogs: {LOG_DIR}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
