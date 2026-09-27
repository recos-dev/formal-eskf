#!/usr/bin/env bash
# Clone the tested PX4 revision, initialize SITL dependencies and copy the package.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_COMMIT=54f0455ffcd755534539a7cf33a09a20bf71d29d

main()
{
    if [[ "${1:-}" == --help ]]; then
        echo "Usage: $0 /path/to/PX4-Autopilot [--gazebo]"
        echo "--gazebo also downloads PX4's pinned Gazebo models/worlds."
        return 0
    fi
    if [[ $# -lt 1 || $# -gt 2 || ( $# == 2 && "$2" != --gazebo ) ]]; then
        echo "Usage: $0 /path/to/PX4-Autopilot [--gazebo]" >&2
        return 1
    fi
    local PX4_ROOT="$1"
    if [[ ! -e "${PX4_ROOT}" ]]; then
        git clone --depth 1 --branch v1.16.2 --no-recurse-submodules \
            https://github.com/PX4/PX4-Autopilot.git "${PX4_ROOT}"
    fi
    if [[ "$(git -C "${PX4_ROOT}" rev-parse HEAD)" != "${PX4_COMMIT}" ]]; then
        echo "PX4 checkout must be ${PX4_COMMIT}; existing work was preserved." >&2
        return 1
    fi
    git -C "${PX4_ROOT}" submodule update --init --recursive --depth 1 \
        src/modules/mavlink/mavlink src/lib/events/libevents src/lib/heatshrink/heatshrink
    if [[ "${2:-}" == --gazebo ]]; then
        git -C "${PX4_ROOT}" submodule update --init --recursive --depth 1 Tools/simulation/gz
    fi
    python3 "${SCRIPT_DIR}/install.py" --px4 "${PX4_ROOT}"
}

main "$@"
