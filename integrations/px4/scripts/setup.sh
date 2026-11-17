#!/usr/bin/env bash
# Clone the tested PX4 revision, initialize SITL dependencies and copy the package.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_COMMIT=54f0455ffcd755534539a7cf33a09a20bf71d29d

usage()
{
    echo "Usage: $0 /path/to/PX4-Autopilot"
    echo "Clone the pinned checkout if missing, fetch SITL dependencies, then install formal-eskf."
}

prepare_checkout()
{
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
}

main()
{
    if [[ $# == 1 && "$1" == --help ]]; then
        usage
        return 0
    fi
    if [[ $# != 1 ]]; then
        usage >&2
        return 1
    fi
    prepare_checkout "$1"
    python3 "${SCRIPT_DIR}/install.py" --px4 "$1"
}

main "$@"
