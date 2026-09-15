#!/usr/bin/env bash

set -euo pipefail

usage()
{
    printf 'Usage: %s {format | checks | cppcheck | cpp | asan | --help}\n' "${0##*/}"
    printf 'Install dependencies for one CI job on Ubuntu 24.04 using sudo apt-get.\n'
}

install_dependencies()
{
    local GROUP="$1"
    local -a PACKAGES=(cmake g++-11 libeigen3-dev)

    case "${GROUP}" in
        format) PACKAGES=(clang-format-14) ;;
        checks) PACKAGES=(shellcheck python3-jsonschema jq ripgrep clang-18 python3-clang-18) ;;
        cppcheck) PACKAGES=(cmake g++ libeigen3-dev make python3 curl) ;;
        cpp) PACKAGES+=(clang-14) ;;
        asan) ;;
        *)
            printf 'error: unknown dependency group: %s\n' "${GROUP}" >&2
            usage >&2
            return 2
            ;;
    esac

    sudo apt-get update
    sudo apt-get install --no-install-recommends -y "${PACKAGES[@]}"
}

main()
{
    if (($# != 1)); then
        usage >&2
        exit 2
    fi
    case "$1" in
        --help|-h) usage ;;
        *) install_dependencies "$1" ;;
    esac
}

main "$@"
