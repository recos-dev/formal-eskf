#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LEAN_DIR="${REPO_DIR}/formal/lean"
ELAN_DIR="${FORMAL_ESKF_ELAN_HOME:-${REPO_DIR}/tools/elan}"
ELAN_VERSION="v4.2.4"
ELAN_ARCHIVE_NAME="elan-x86_64-unknown-linux-gnu.tar.gz"
ELAN_ARCHIVE_SHA256="42b94d4244e8353142c456ec0e4ca6528fd898a6c604d4059f494e706e431f63"
BOOTSTRAP_DIR="${REPO_DIR}/tools/bootstrap/lean"
ELAN_ARCHIVE="${BOOTSTRAP_DIR}/${ELAN_ARCHIVE_NAME}"
LAKE_COMMAND=""

fail()
{
    printf 'error: %s\n' "$*" >&2
    exit 127
}

check_bootstrap_environment()
{
    local PLATFORM REQUIRED_TOOL
    PLATFORM="$(uname -s):$(uname -m)"

    if [[ "${PLATFORM}" != "Linux:x86_64" ]]; then
        fail "automatic Elan bootstrap supports Linux x86_64 only; install Elan manually"
    fi

    for REQUIRED_TOOL in curl tar sha256sum; do
        command -v "${REQUIRED_TOOL}" >/dev/null 2>&1 ||
            fail "${REQUIRED_TOOL} is required to bootstrap Elan"
    done
}

bootstrap_elan()
{
    check_bootstrap_environment

    printf 'lake was not found; bootstrapping Elan %s locally\n' "${ELAN_VERSION}"
    mkdir -p -- "${BOOTSTRAP_DIR}" "${ELAN_DIR}"
    curl -L --fail --show-error \
        --output "${ELAN_ARCHIVE}" \
        "https://github.com/leanprover/elan/releases/download/${ELAN_VERSION}/${ELAN_ARCHIVE_NAME}"
    printf '%s  %s\n' "${ELAN_ARCHIVE_SHA256}" "${ELAN_ARCHIVE}" | sha256sum --check --status
    tar -xzf "${ELAN_ARCHIVE}" -C "${BOOTSTRAP_DIR}"
    ELAN_HOME="${ELAN_DIR}" "${BOOTSTRAP_DIR}/elan-init" \
        -y --default-toolchain none --no-modify-path

    LAKE_COMMAND="${ELAN_DIR}/bin/lake"
}

select_lake()
{
    if [[ -x "${ELAN_DIR}/bin/lake" ]]; then
        LAKE_COMMAND="${ELAN_DIR}/bin/lake"
    elif command -v lake >/dev/null 2>&1; then
        LAKE_COMMAND="$(command -v lake)"
    else
        bootstrap_elan
    fi
}

build_lean()
{
    cd -- "${LEAN_DIR}"
    if [[ "${LAKE_COMMAND}" == "${ELAN_DIR}/bin/lake" ]]; then
        ELAN_HOME="${ELAN_DIR}" "${LAKE_COMMAND}" build FormalESKF
    else
        "${LAKE_COMMAND}" build FormalESKF
    fi
}

main()
{
    select_lake
    build_lean
}

main "$@"
