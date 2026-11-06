#!/usr/bin/env bash
# Run the existing core suite with the PX4 deployment matrix backend.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

main()
{
    if [[ $# -lt 1 || $# -gt 2 || "${1:-}" == --help ]]; then
        echo "Usage: $0 /path/to/PX4-Autopilot [test-build-directory]"
        return 0
    fi
    local PX4_ROOT PX4_BUILD TEST_BUILD INCLUDES
    PX4_ROOT="$(cd -- "$1" && pwd)"
    PX4_BUILD="${PX4_ROOT}/build/px4_sitl_formal_eskf"
    TEST_BUILD="${2:-${REPO_DIR}/build/core-px4}"
    INCLUDES="${PX4_ROOT}/src/lib/matrix;${PX4_ROOT}/platforms/common/include;${PX4_ROOT}/platforms/posix/include;${PX4_BUILD};${PX4_ROOT}/src/include"
    cmake -S "${REPO_DIR}" -B "${TEST_BUILD}" \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
        -DFORMAL_ESKF_BUILD_EIGEN_BACKEND=OFF \
        -DFORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND=ON \
        -DFORMAL_ESKF_PX4_INCLUDE_DIRS="${INCLUDES}"
    cmake --build "${TEST_BUILD}" --parallel "${JOBS:-4}"
    ctest --test-dir "${TEST_BUILD}" --output-on-failure
}

main "$@"
