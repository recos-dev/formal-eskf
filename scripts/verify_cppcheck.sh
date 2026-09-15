#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_DIR}/build/cppcheck"
CONFIG_FILE="${REPO_DIR}/configs/cppcheck.cppcheck"

CPPCHECK_VERSION="2.21.0"
CPPCHECK_DIR="${REPO_DIR}/tools/cppcheck-${CPPCHECK_VERSION}"
CPPCHECK_SOURCE_DIR="${CPPCHECK_DIR}/source"
LOCAL_CPPCHECK="${CPPCHECK_SOURCE_DIR}/cppcheck"
CPPCHECK_COMMAND="${FORMAL_ESKF_CPPCHECK:-}"

ARCHIVE_NAME="cppcheck-${CPPCHECK_VERSION}.tar.gz"
ARCHIVE_SHA256="f028ff75ca5372738f3737c8b3e8611426a6526b6aea2ef01301ab0f5902f044"
ARCHIVE_URL="https://github.com/cppcheck-opensource/cppcheck/archive/refs/tags/${CPPCHECK_VERSION}.tar.gz"
BOOTSTRAP_DIR="${REPO_DIR}/tools/bootstrap/cppcheck"
ARCHIVE_FILE="${BOOTSTRAP_DIR}/${ARCHIVE_NAME}"
ARCHIVE_PART="${ARCHIVE_FILE}.part"

CPPCHECK_PROJECT_ARGUMENTS=(
    --project="${CONFIG_FILE}"
    --std=c++20
    --relative-paths="${REPO_DIR}"
    --template=gcc
)
CPPCHECK_ANALYSIS_ARGUMENTS=(
    "--enable=warning,style,performance,portability"
    --inline-suppr
    --safety
    --error-exitcode=2
)
EXPECTED_ACTIVE_CHECKERS=173
EXPECTED_INACTIVE_CHECKERS=(
    CheckBufferOverrun::stringNotZeroTerminated
    CheckClass::initializerListOrder
    CheckExceptionSafety::unhandledExceptionSpecification
    CheckMemoryLeakNoVar::checkForUnsafeArgAlloc
    CheckOther::checkDuplicateBranch
    CheckOther::checkIncompleteArrayFill
    CheckOther::checkInterlockedDecrement
    CheckOther::checkRedundantCopy
    CheckOther::checkSuspiciousCaseInSwitch
    CheckOther::checkSuspiciousSemicolon
    CheckSizeof::suspiciousSizeofCalculation
    CheckStl::size
    CheckUnusedFunctions::check
)

usage()
{
    printf 'Usage: %s [--help]\n' "${0##*/}"
    printf 'Check C++ sources on unix32 and unix64 without modifying source files.\n'
    printf 'Bootstraps the pinned Cppcheck version if it is not available.\n'
    printf 'Set FORMAL_ESKF_CPPCHECK to use an existing executable of that version.\n'
}

parse_arguments()
{
    if (($# == 0)); then
        return
    fi
    if (($# == 1)) && [[ "$1" == --help || "$1" == -h ]]; then
        usage
        exit 0
    fi
    printf 'error: unexpected arguments: %s\n' "$*" >&2
    usage >&2
    exit 2
}

fail()
{
    printf 'error: %s\n' "$*" >&2
    exit 127
}

check_bootstrap_environment()
{
    local REQUIRED_TOOL

    for REQUIRED_TOOL in curl g++ make sha256sum tar; do
        command -v "${REQUIRED_TOOL}" >/dev/null 2>&1 ||
            fail "${REQUIRED_TOOL} is required to bootstrap Cppcheck"
    done
}

bootstrap_cppcheck()
{
    local BUILD_JOBS

    check_bootstrap_environment

    mkdir -p -- "${BOOTSTRAP_DIR}" "${CPPCHECK_SOURCE_DIR}"
    if [[ ! -f "${ARCHIVE_FILE}" ]]; then
        printf 'Cppcheck %s was not found; downloading the pinned source archive\n' "${CPPCHECK_VERSION}"
        curl -L --fail --show-error --output "${ARCHIVE_PART}" "${ARCHIVE_URL}"
        printf '%s  %s\n' "${ARCHIVE_SHA256}" "${ARCHIVE_PART}" | sha256sum --check --status ||
            fail "downloaded Cppcheck source archive checksum does not match"
        mv -- "${ARCHIVE_PART}" "${ARCHIVE_FILE}"
    fi

    printf '%s  %s\n' "${ARCHIVE_SHA256}" "${ARCHIVE_FILE}" | sha256sum --check --status ||
        fail "Cppcheck source archive checksum does not match"

    if [[ ! -f "${CPPCHECK_SOURCE_DIR}/Makefile" ]]; then
        tar -xzf "${ARCHIVE_FILE}" --strip-components=1 -C "${CPPCHECK_SOURCE_DIR}"
    fi

    BUILD_JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '2')"
    if ((BUILD_JOBS > 4)); then
        BUILD_JOBS=4
    fi
    printf 'building Cppcheck %s locally\n' "${CPPCHECK_VERSION}"
    make -s -C "${CPPCHECK_SOURCE_DIR}" -j "${BUILD_JOBS}" \
        "FILESDIR=${CPPCHECK_SOURCE_DIR}" \
        "CFGDIR=${CPPCHECK_SOURCE_DIR}/cfg"

    CPPCHECK_COMMAND="${LOCAL_CPPCHECK}"
}

select_cppcheck()
{
    local PATH_CPPCHECK PATH_VERSION

    if [[ -n "${CPPCHECK_COMMAND}" ]]; then
        return
    fi
    if [[ -x "${LOCAL_CPPCHECK}" ]]; then
        CPPCHECK_COMMAND="${LOCAL_CPPCHECK}"
        return
    fi
    if command -v cppcheck >/dev/null 2>&1; then
        PATH_CPPCHECK="$(command -v cppcheck)"
        PATH_VERSION="$("${PATH_CPPCHECK}" --version 2>/dev/null || true)"
        if [[ "${PATH_VERSION}" == "Cppcheck ${CPPCHECK_VERSION}" ]]; then
            CPPCHECK_COMMAND="${PATH_CPPCHECK}"
            return
        fi
        printf 'ignoring %s; this analysis profile requires Cppcheck %s\n' \
            "${PATH_VERSION:-unknown Cppcheck version}" "${CPPCHECK_VERSION}"
    fi

    bootstrap_cppcheck
}

check_cppcheck_version()
{
    local ACTUAL_VERSION

    [[ -x "${CPPCHECK_COMMAND}" ]] ||
        fail "Cppcheck is not executable: ${CPPCHECK_COMMAND}"

    ACTUAL_VERSION="$("${CPPCHECK_COMMAND}" --version 2>/dev/null || true)"
    [[ "${ACTUAL_VERSION}" == "Cppcheck ${CPPCHECK_VERSION}" ]] ||
        fail "expected Cppcheck ${CPPCHECK_VERSION}, found ${ACTUAL_VERSION:-unknown version}"
}

check_runtime_environment()
{
    command -v python3 >/dev/null 2>&1 || fail "python3 is required by the Cppcheck threadsafety addon"
}

configure_analysis()
{
    command -v cmake >/dev/null 2>&1 || fail "cmake is required to generate the compile database"

    printf '[1/3] Configure C++ analysis\n'
    cmake \
        -S "${REPO_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DBUILD_TESTING=ON \
        -DFORMAL_ESKF_BUILD_EIGEN_BACKEND=ON
}

analyze_platform()
{
    local PLATFORM="$1"
    local CACHE_DIR="${BUILD_DIR}/cache/${PLATFORM}"
    local REPORT_FILE="${BUILD_DIR}/checkers-${PLATFORM}.txt"

    mkdir -p -- "${CACHE_DIR}"
    "${CPPCHECK_COMMAND}" \
        "${CPPCHECK_PROJECT_ARGUMENTS[@]}" \
        "${CPPCHECK_ANALYSIS_ARGUMENTS[@]}" \
        --platform="${PLATFORM}" \
        --cppcheck-build-dir="${CACHE_DIR}" \
        --checkers-report="${REPORT_FILE}"
}

check_configuration()
{
    local PLATFORM="$1"

    "${CPPCHECK_COMMAND}" \
        "${CPPCHECK_PROJECT_ARGUMENTS[@]}" \
        --platform="${PLATFORM}" \
        --check-config \
        --error-exitcode=2
}

check_checker_coverage()
{
    local PLATFORM="$1"
    local REPORT_FILE="${BUILD_DIR}/checkers-${PLATFORM}.txt"
    local ACTIVE_CHECKERS INDEX
    local -a ACTUAL_INACTIVE_CHECKERS

    [[ -f "${REPORT_FILE}" ]] || {
        printf 'error: Cppcheck did not write %s\n' "${REPORT_FILE}" >&2
        return 127
    }

    ACTIVE_CHECKERS="$(awk '$1 == "Yes" && $2 ~ /::/ { COUNT++ } END { print COUNT + 0 }' "${REPORT_FILE}")"
    mapfile -t ACTUAL_INACTIVE_CHECKERS < <(awk '$1 == "No" && $2 ~ /::/ { print $2 }' "${REPORT_FILE}")

    if [[ "${ACTIVE_CHECKERS}" -ne "${EXPECTED_ACTIVE_CHECKERS}" ]]; then
        printf 'error: expected %d active Cppcheck checkers, found %d for %s\n' \
            "${EXPECTED_ACTIVE_CHECKERS}" "${ACTIVE_CHECKERS}" "${PLATFORM}" >&2
        return 2
    fi
    if [[ "${#ACTUAL_INACTIVE_CHECKERS[@]}" -ne "${#EXPECTED_INACTIVE_CHECKERS[@]}" ]]; then
        printf 'error: unexpected inactive Cppcheck checker count for %s\n' "${PLATFORM}" >&2
        return 2
    fi

    for INDEX in "${!EXPECTED_INACTIVE_CHECKERS[@]}"; do
        if [[ "${ACTUAL_INACTIVE_CHECKERS[INDEX]}" != "${EXPECTED_INACTIVE_CHECKERS[INDEX]}" ]]; then
            printf 'error: unexpected inactive Cppcheck checker %s for %s\n' \
                "${ACTUAL_INACTIVE_CHECKERS[INDEX]}" "${PLATFORM}" >&2
            return 2
        fi
    done

    printf 'Checker coverage: %d active, %d intentionally inactive\n' \
        "${ACTIVE_CHECKERS}" "${#ACTUAL_INACTIVE_CHECKERS[@]}"
}

verify_platform()
{
    local PLATFORM="$1"
    local ANALYSIS_RESULT=0

    printf '  Configuration\n'
    check_configuration "${PLATFORM}" || return $?
    printf '  Analysis\n'
    analyze_platform "${PLATFORM}" || ANALYSIS_RESULT=$?
    check_checker_coverage "${PLATFORM}" || return $?

    return "${ANALYSIS_RESULT}"
}

run_analysis()
{
    local RESULT=0

    printf '[2/3] Cppcheck unix32\n'
    verify_platform unix32 || RESULT=$?

    printf '[3/3] Cppcheck unix64\n'
    verify_platform unix64 || RESULT=$?

    return "${RESULT}"
}

main()
{
    parse_arguments "$@"
    cd -- "${REPO_DIR}"
    select_cppcheck
    check_cppcheck_version
    check_runtime_environment
    configure_analysis
    run_analysis
}

main "$@"
