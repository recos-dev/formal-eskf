#!/usr/bin/env python3
"""Host compile checks for declared shapes, configuration and layer boundaries."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

REPO_DIR = Path(__file__).resolve().parents[2]
PROFILE = "host-eigen"
CASES = (
    "TC-F-LINALG-SHAPES",
    "TC-C-CPP-SUBSET-CONFIGURATION",
    "TC-C-LAYER-BOUNDARY-DEPENDENCIES",
    "TC-C-CPP-SUBSET-RESTRICTIONS",
    "TC-F-GEO-ECEF-FORMAT",
)
PREAMBLE = """
#include <formal_eskf/linalg/backend/eigen.hpp>
#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/ring_buffer.hpp>
#include <formal_eskf/runtime/accelerometer.hpp>
#include <limits>
using Backend = formal_eskf::linalg::EigenBackend<double>;
template <std::size_t R, std::size_t C> using Matrix = formal_eskf::linalg::Matrix<Backend, R, C>;
struct Sample { std::uint64_t time_us{}; int value{}; };
"""


def compile_source(ARGS, DIRECTORY, SOURCE, *, DEFINITIONS=(), EXPECT_SUCCESS=True, OPTIONS=(), DIAGNOSTIC=None):
    SOURCE_FILE = DIRECTORY / "fixture.cpp"
    SOURCE_FILE.write_text(SOURCE)
    COMMAND = [ARGS.compiler, *ARGS.compiler_arg, "-std=c++20", "-fno-exceptions", "-fno-rtti", "-fno-fast-math",
               "-ffp-contract=off", "-I", str(REPO_DIR / "include")]
    for INCLUDE in ARGS.include_dir:
        COMMAND += ["-I", INCLUDE]
    COMMAND += [f"-D{VALUE}" for VALUE in DEFINITIONS]
    COMMAND += list(OPTIONS or ("-fsyntax-only",)) + [str(SOURCE_FILE)]
    RESULT = subprocess.run(COMMAND, capture_output=True, text=True, timeout=120, check=False)
    if (RESULT.returncode == 0) != EXPECT_SUCCESS:
        raise AssertionError(f"unexpected compile result ({RESULT.returncode}): {shlex.join(COMMAND)}\n{RESULT.stderr}")
    if not EXPECT_SUCCESS and RESULT.returncode < 0:
        raise AssertionError("compiler termination is not an expected compile-time rejection")
    if DIAGNOSTIC and DIAGNOSTIC not in RESULT.stderr:
        raise AssertionError(f"missing expected diagnostic: {DIAGNOSTIC}\n{RESULT.stderr}")
    return RESULT.stdout


def shapes(ARGS, DIRECTORY):
    SHAPES = [(N, 1) for N in (1, 2, 3, 4, 6, 15, 16)] + [
        (1, 3), (1, 15), (2, 2), (2, 3), (3, 2), (2, 15), (2, 16), (15, 2),
        (3, 3), (3, 15), (3, 16), (15, 3), (4, 3), (4, 4), (4, 15), (4, 16), (15, 4),
        (6, 3), (3, 6), (6, 6), (6, 15), (6, 16), (15, 6), (12, 12), (12, 15), (15, 12), (15, 15), (16, 15)]
    BODY = "\n".join(f"Matrix<{R},{C}> m{I}; (void)m{I};" for I, (R, C) in enumerate(SHAPES))
    compile_source(ARGS, DIRECTORY, PREAMBLE + "int main(){" + BODY + "}")
    # Compile each valid companion immediately before its prohibited operation.
    # A broken include/compiler setup cannot make a negative check pass.
    PAIRS = (
        ("Matrix<1,1> m;", "Matrix<0,1> m;"),
        ("Matrix<1,1> m;", "Matrix<1,0> m;"),
        ("auto m=Matrix<3,3>::identity();", "auto m=Matrix<2,3>::identity();"),
        ("Matrix<3,1> m; (void)m(0);", "Matrix<3,2> m; (void)m(0);"),
        ("Matrix<2,3> a; Matrix<3,2> b; (void)(a*b);", "Matrix<2,3> a; Matrix<2,3> b; (void)(a*b);"),
        ("Matrix<3,3> m; (void)m.block<1,1,2,2>();", "Matrix<3,3> m; (void)m.block<2,1,2,2>();"),
        ("Matrix<3,3> m; (void)m.block<1,1,2,2>();", "Matrix<3,3> m; (void)m.block<1,2,2,2>();"),
        ("Matrix<3,3> m; (void)m.block<0,0,1,1>();", "Matrix<3,3> m; (void)m.block<std::numeric_limits<std::size_t>::max(),0,2,1>();"),
        ("Matrix<3,3> m; m.set_block<1,1>(Matrix<2,2>{});", "Matrix<3,3> m; m.set_block<2,2>(Matrix<2,2>{});"),
        ("Matrix<3,1> m; (void)m.segment<1,2>();", "Matrix<3,1> m; (void)m.segment<2,2>();"),
        ("Matrix<3,1> m; m.set_segment<1>(Matrix<2,1>{});", "Matrix<3,1> m; m.set_segment<2>(Matrix<2,1>{});"),
        ("formal_eskf::runtime::RingBuffer<Sample,1> b(1);", "formal_eskf::runtime::RingBuffer<Sample,0> b(1);"),
    )
    for VALID, INVALID in PAIRS:
        compile_source(ARGS, DIRECTORY, PREAMBLE + "int main(){" + VALID + "}")
        compile_source(ARGS, DIRECTORY, PREAMBLE + "int main(){" + INVALID + "}", EXPECT_SUCCESS=False)


def configuration(ARGS, DIRECTORY):
    DEFAULTS = {"ESKF_CODEGEN": 1, "ESKF_QUAT_APPROX": 0, "ESKF_RESET_APPROX": 0, "ESKF_MAG_TILT": 1}
    ASSERTIONS = "\n".join(f"static_assert({NAME}=={VALUE});" for NAME, VALUE in DEFAULTS.items())
    compile_source(ARGS, DIRECTORY, PREAMBLE + ASSERTIONS)
    compile_source(ARGS, DIRECTORY, PREAMBLE + ASSERTIONS,
                   DEFINITIONS=[f"{NAME}={VALUE}" for NAME, VALUE in DEFAULTS.items()])
    for NAME in DEFAULTS:
        for VALUE in (0, 1):
            compile_source(ARGS, DIRECTORY, PREAMBLE + f"static_assert({NAME}=={VALUE});",
                           DEFINITIONS=[f"{NAME}={VALUE}"])
        for VALUE in (-1, 2):
            compile_source(ARGS, DIRECTORY, PREAMBLE, DEFINITIONS=[f"{NAME}={VALUE}"], EXPECT_SUCCESS=False)


def dependencies(ARGS, DIRECTORY):
    INCLUDE_DIR = REPO_DIR / "include" / "formal_eskf"
    HEADERS = sorted((INCLUDE_DIR / "eskf").glob("*.hpp")) + sorted((INCLUDE_DIR / "runtime").glob("*.hpp"))
    HEADERS += sorted((INCLUDE_DIR / "geodesy").glob("*.hpp"))
    if not HEADERS:
        raise AssertionError("empty public-header inventory")
    for HEADER in HEADERS:
        NAME = HEADER.relative_to(REPO_DIR / "include").as_posix()
        SOURCE = f"#include <{NAME}>\nint main() {{}}\n"
        compile_source(ARGS, DIRECTORY, SOURCE)
        DEPENDENCIES = compile_source(ARGS, DIRECTORY, SOURCE, OPTIONS=("-MM", "-MT", "fixture"))
        TOKENS = shlex.split(DEPENDENCIES.replace("\\\n", " "))[1:]
        FORBIDDEN = ("integrations/", "examples/", "uORB/", "px4_platform_common/", "nuttx/")
        if HEADER.parent.name in ("eskf", "geodesy"):
            FORBIDDEN += ("formal_eskf/runtime/",)
        if HEADER.parent.name == "geodesy":
            FORBIDDEN += ("formal_eskf/eskf/",)
        BAD = [TOKEN for TOKEN in TOKENS if any(PART in TOKEN for PART in FORBIDDEN)]
        if BAD:
            raise AssertionError(f"{NAME} has forbidden transitive dependencies: {BAD}")


def restrictions(ARGS, DIRECTORY):
    # Representative instantiated operations, not a static whole-program proof.
    SOURCE = PREAMBLE + """
using Types = formal_eskf::EskfTypes<Backend,formal_eskf::configuration::Ins>;
formal_eskf::Status step(Types::nominal_state_type & state, Types::imu_sample_type const & imu,
                        Types::parameter_type const & parameters, double dt) {
    return formal_eskf::predict_nominal(state,imu,dt,parameters,state);
}
bool history(formal_eskf::runtime::RingBuffer<Sample,8> & buffer, Sample const & sample, Sample & output) {
    buffer.push(sample); return buffer.pop_first_older_than(sample.time_us,&output);
}
auto magnitude(formal_eskf::runtime::AccelerometerMagnitude<Backend> & filter, Types::vector3_type const & sample) {
    return filter.update(sample);
}
""".replace("Types::vector3_type", "formal_eskf::linalg::Vector<Backend,3>")
    OBJECT = DIRECTORY / "restrictions.o"
    compile_source(ARGS, DIRECTORY, SOURCE, OPTIONS=("-O2", "-fstack-usage", "-c", "-o", str(OBJECT)))
    RESULT = subprocess.run(["nm", "-u", str(OBJECT)], text=True, capture_output=True, check=True, timeout=30)
    SYMBOLS = [LINE.split()[-1] for LINE in RESULT.stdout.splitlines() if LINE.split()]
    FORBIDDEN = ("malloc", "calloc", "realloc", "free", "_Zn", "_Zdl", "__cxa_throw", "pthread_", "mutex", "sleep")
    BAD = [NAME for NAME in SYMBOLS if NAME.startswith(FORBIDDEN)]
    if BAD:
        raise AssertionError(f"representative object has forbidden runtime dependencies: {BAD}")
    REPORTS = list(DIRECTORY.glob("*.su"))
    if not REPORTS or not any(P.read_text().strip() for P in REPORTS):
        raise AssertionError("compiler supplied no stack-usage report")
    print("Scope: representative object dependencies and compiler stack report; no target budget claim")
    for REPORT in REPORTS:
        print(REPORT.read_text(), end="")


def geodesy_format(ARGS, DIRECTORY):
    PREFIX = """
#include <formal_eskf/geodesy/local_ned.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>
namespace geo = formal_eskf::geodesy;
"""
    OPERATIONS = (
        "geo::LocalNed<B> frame; (void)frame;",
        "geo::GeodeticPosition g; formal_eskf::linalg::Vector<B,3> e; (void)geo::geodetic_to_ecef<B>(g,e);",
        "geo::GeodeticPosition g; formal_eskf::linalg::Vector<B,3> e; (void)geo::ecef_to_geodetic<B>(e,g);",
    )
    for OPERATION in OPERATIONS:
        for SCALAR in ("double", "float"):
            SOURCE = PREFIX + f"using B = formal_eskf::linalg::EigenBackend<{SCALAR}>;\nint main(){{{OPERATION}}}"
            compile_source(ARGS, DIRECTORY, SOURCE, EXPECT_SUCCESS=SCALAR == "double",
                           DIAGNOSTIC=None if SCALAR == "double" else "Geodesy requires IEEE-754 binary64 double")


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
    PARSER.add_argument("--include-dir", action="append", default=[])
    PARSER.add_argument("--compiler-arg", action="append", default=[])
    PARSER.add_argument("--list", action="store_true")
    PARSER.add_argument("--case", choices=CASES)
    PARSER.add_argument("--profile", choices=[PROFILE])
    ARGS = PARSER.parse_args()
    if bool(ARGS.case) != bool(ARGS.profile) or (ARGS.list and ARGS.case):
        PARSER.error("select --case and --profile together, or use --list")
    if ARGS.list:
        for CASE in CASES:
            print(CASE, PROFILE)
        return 0
    if not ARGS.include_dir:
        ARGS.include_dir = ["/usr/include/eigen3"]
    OUTPUT_DIR = REPO_DIR / "output" / "test-compile-contracts"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    FAILED = 0
    for CASE, CALLBACK in zip(CASES, (shapes, configuration, dependencies, restrictions, geodesy_format)):
        if ARGS.case and CASE != ARGS.case:
            continue
        print(f"RUN {CASE} {PROFILE}", flush=True)
        try:
            with tempfile.TemporaryDirectory(prefix="check-", dir=OUTPUT_DIR) as TEMP:
                CALLBACK(ARGS, Path(TEMP))
        except (AssertionError, OSError, subprocess.SubprocessError) as ERROR:
            FAILED += 1
            print(ERROR)
            print(f"FAIL {CASE} {PROFILE}", flush=True)
        else:
            print(f"PASS {CASE} {PROFILE}", flush=True)
    return int(FAILED != 0)


if __name__ == "__main__":
    raise SystemExit(main())
