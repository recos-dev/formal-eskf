#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_DIR}/build/sanitizers"
BUILD_JOBS="${BUILD_JOBS:-2}"
TEST_REGEX=""
SANITIZER_FLAGS='-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer'
PASS_COLOR='\e[32;01m'
FAIL_COLOR='\e[31;01m'
NO_COLOR='\e[0m'

usage()
{
    printf 'Usage: %s [TEST_REGEX | --help]\n' "${0##*/}"
    printf 'Build Debug tests with ASan and UBSan, then run CTest.\n'
    printf 'Uses build/sanitizers and the default ESKF approximation settings (OFF).\n'
    printf '  TEST_REGEX     Run matching CTest names only; default: all tests.\n'
    printf '  BUILD_JOBS     Parallel compilation jobs (default: 2).\n'
    printf '  ASAN_OPTIONS   Runtime options (default: detect_leaks=1).\n'
    printf '  UBSAN_OPTIONS  Runtime options (default: print_stacktrace=1:halt_on_error=1).\n'
    printf '\nFor environments that cannot run LeakSanitizer (such as ptrace):\n'
    printf '  ASAN_OPTIONS=detect_leaks=0 %s\n' "$0"
    printf 'Disabling leak detection does not disable address/undefined-behavior checks.\n'
}

fail()
{
    printf '%berror: %s%b\n' "${FAIL_COLOR}" "$*" "${NO_COLOR}" >&2
    exit 2
}

parse_arguments()
{
    (($# <= 1)) || fail 'expected at most one test regex; use --help for usage'
    case "${1:-}" in
        --help|-h) usage; exit 0 ;;
        -*) fail "unknown option: $1; use --help for usage" ;;
        *) TEST_REGEX="${1:-}" ;;
    esac
}

report_result()
{
    local RESULT=$?

    if ((RESULT == 0)); then
        printf 'ASan/UBSan: %bpass%b\n' "${PASS_COLOR}" "${NO_COLOR}"
    else
        printf 'ASan/UBSan: %bfail%b (exit %d)\n' "${FAIL_COLOR}" "${NO_COLOR}" "${RESULT}" >&2
    fi
}

check_environment()
{
    local REQUIRED_TOOL

    for REQUIRED_TOOL in cmake ctest; do
        command -v "${REQUIRED_TOOL}" >/dev/null 2>&1 || fail "${REQUIRED_TOOL} was not found in PATH"
    done
    [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || fail 'BUILD_JOBS must be a positive integer'
}

configure_build()
{
    printf '[1/3] Configure ASan/UBSan\n'
    # Keep instrumentation out of normal builds. Debug also keeps the Eigen
    # runtime no-allocation assertions active. Disable UBSan recovery so a
    # diagnostic cannot be followed by a successful CTest exit status.
    cmake \
        -S "${REPO_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS:STRING="${SANITIZER_FLAGS}" \
        -DCMAKE_CXX_FLAGS_DEBUG:STRING=-g \
        -DBUILD_TESTING=ON \
        -DFORMAL_ESKF_BUILD_EIGEN_BACKEND=ON \
        -DESKF_QUAT_APPROX=OFF \
        -DESKF_RESET_APPROX=OFF
}

build_tests()
{
    printf '[2/3] Build ASan/UBSan tests\n'
    cmake --build "${BUILD_DIR}" --config Debug --parallel "${BUILD_JOBS}"
}

run_tests()
{
    local -a TEST_ARGUMENTS=(--test-dir "${BUILD_DIR}" --build-config Debug --output-on-failure --no-tests=error)

    if [[ -n "${TEST_REGEX}" ]]; then
        TEST_ARGUMENTS+=(--tests-regex "${TEST_REGEX}")
    fi
    export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1}"
    export UBSAN_OPTIONS="${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=1}"
    printf '[3/3] Run ASan/UBSan tests\n'
    printf 'ASAN_OPTIONS=%s\n' "${ASAN_OPTIONS}"
    printf 'UBSAN_OPTIONS=%s\n' "${UBSAN_OPTIONS}"
    ctest "${TEST_ARGUMENTS[@]}"
}

main()
{
    parse_arguments "$@"
    trap report_result EXIT
    check_environment
    configure_build
    build_tests
    run_tests
}

main "$@"
