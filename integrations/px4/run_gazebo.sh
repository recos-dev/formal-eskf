#!/usr/bin/env bash
# Build formal-eskf, fly a Gazebo X500 and analyze the original ULog.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
PX4_ROOT="${REPO_ROOT}/build/px4-gazebo/PX4-Autopilot"
VENV_DIR="${REPO_ROOT}/build/px4-gazebo/venv"
WORLD=default
HEADLESS_MODE=0
SETUP=0
JOBS=4
OUTPUT=""

usage()
{
    cat <<EOF
Usage: $0 [options]

Opens Gazebo, normally arms the X500, flies a square, lands and closes the
simulation. formal-eskf replaces EKF2. Saves ULog, provenance and flight.png.

  --setup             Fetch pinned PX4/models and install Python dependencies
                      into ${VENV_DIR} (requires network)
  --px4 PATH          Existing PX4 v1.16.2 checkout (default: ${PX4_ROOT})
  --headless          Run without the Gazebo window
  --world NAME        default or windy (default: default)
  --jobs N            Parallel build jobs (default: 4)
  --output PATH       New directory for flight evidence
  --help              Show this help

Requires Gazebo Harmonic, C++ build tools and Python 3 with venv support.
Ubuntu instructions: ${SCRIPT_DIR}/README.md
Existing Python environment: set PYTHON=/path/to/venv/bin/python.
EOF
}

parse_arguments()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help) usage; exit 0 ;;
            --setup) SETUP=1; shift ;;
            --headless) HEADLESS_MODE=1; shift ;;
            --px4|--world|--jobs|--output)
                if [[ $# -lt 2 || "$2" == --* ]]; then
                    echo "Missing value for $1" >&2
                    exit 1
                fi
                case "$1" in
                    --px4) PX4_ROOT="$2" ;;
                    --world) WORLD="$2" ;;
                    --jobs) JOBS="$2" ;;
                    --output) OUTPUT="$2" ;;
                esac
                shift 2
                ;;
            *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        esac
    done
    if [[ "$WORLD" != default && "$WORLD" != windy ]]; then
        echo "--world must be default or windy" >&2
        exit 1
    fi
    if [[ ! "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
        echo "--jobs must be a positive integer" >&2
        exit 1
    fi
    if [[ "$HEADLESS_MODE" == 0 && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        echo "No graphical display. Run from your desktop terminal, or use --headless." >&2
        exit 1
    fi
    if [[ -n "$OUTPUT" && -e "$OUTPUT" ]]; then
        echo "--output must name a new directory; existing evidence was preserved." >&2
        exit 1
    fi
}

prepare()
{
    local TOOL
    for TOOL in git cmake ninja make g++ gz python3; do
        if ! command -v "$TOOL" >/dev/null; then
            echo "Missing $TOOL; see the Gazebo setup section in ${SCRIPT_DIR}/README.md" >&2
            exit 1
        fi
    done
    local GAZEBO_VERSIONS
    GAZEBO_VERSIONS="$(gz sim --versions)"
    if [[ "${GAZEBO_VERSIONS%%$'\n'*}" != 8.* ]]; then
        echo "Gazebo Harmonic (gz-sim8) must be the active gz sim version." >&2
        exit 1
    fi
    if [[ "$SETUP" == 1 ]]; then
        "${SCRIPT_DIR}/setup.sh" "$PX4_ROOT" --gazebo
        if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
            python3 -m venv "$VENV_DIR"
        fi
        "${VENV_DIR}/bin/python" -m pip install \
            -r "${PX4_ROOT}/Tools/setup/requirements.txt" -r "${SCRIPT_DIR}/requirements.txt"
    fi
    if [[ -z "${PYTHON:-}" ]]; then
        if [[ -x "${VENV_DIR}/bin/python" ]]; then
            PYTHON="${VENV_DIR}/bin/python"
        else
            PYTHON=python3
        fi
    fi
    if [[ ! -f "${PX4_ROOT}/Tools/simulation/gz/models/x500/model.sdf" ]]; then
        echo "PX4 or pinned Gazebo models are missing. Run with --setup first." >&2
        exit 1
    fi
    if ! "$PYTHON" -c 'import em, jinja2, kconfiglib, numpy, matplotlib, pymavlink, pyulog'; then
        echo "Python dependencies missing. Run with --setup or select PYTHON." >&2
        exit 1
    fi
    PX4_ROOT="$(cd -- "$PX4_ROOT" && pwd)"
    PYTHON="$(command -v "$PYTHON")"
    PATH="$(dirname -- "$PYTHON"):$PATH"
    export PATH
    export GZ_DISTRO=harmonic
    "$PYTHON" "${SCRIPT_DIR}/install.py" --px4 "$PX4_ROOT"
    make -C "$PX4_ROOT" px4_sitl_gz_formal_eskf -j"$JOBS"
    "$PYTHON" "${SCRIPT_DIR}/install.py" --px4 "$PX4_ROOT" --check
    if [[ ! -x "${PX4_ROOT}/build/px4_sitl_gz_formal_eskf/bin/px4-gz_bridge" ||
          ! -f "${PX4_ROOT}/build/px4_sitl_gz_formal_eskf/rootfs/gz_env.sh" ]]; then
        echo "Gazebo bridge is missing from the build. Check Gazebo development packages and board configuration." >&2
        exit 1
    fi
}

main()
{
    parse_arguments "$@"
    prepare
    local RUN_DIR="${OUTPUT:-${REPO_ROOT}/build/gazebo-$(date -u +%Y%m%d-%H%M%S)-$$}"
    local GUI_ARGS=()
    if [[ "$HEADLESS_MODE" == 0 ]]; then
        GUI_ARGS=(--gui)
    fi
    echo "Starting formal-eskf Gazebo flight; evidence: ${RUN_DIR}"
    "$PYTHON" "${SCRIPT_DIR}/sitl_flight.py" --px4 "$PX4_ROOT" \
        --simulator gazebo --world "$WORLD" --startup-timeout 180 \
        --output "$RUN_DIR" "${GUI_ARGS[@]}"
    "$PYTHON" "${SCRIPT_DIR}/analyze_flight.py" "$RUN_DIR"
    echo "Flight and ULog checks passed. Open ${RUN_DIR}/flight.png"
}

main "$@"
