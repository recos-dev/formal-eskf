#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${REPO_DIR}/.clang-format"
FORMAT_COMMAND="${CLANG_FORMAT:-clang-format}"
MODE=fix
PASS_COLOR='\e[32;01m'
FAIL_COLOR='\e[31;01m'
NO_COLOR='\e[0m'

usage()
{
    printf 'Usage: %s [--check | --fix | --help]\n' "${0##*/}"
    printf '  --check  Check C++ formatting without modifying files.\n'
    printf '  --fix    Format C++ files in place using .clang-format (default).\n'
    printf 'Set CLANG_FORMAT to select the formatter executable.\n'
}

fail()
{
    printf '%berror: %s%b\n' "${FAIL_COLOR}" "$*" "${NO_COLOR}" >&2
    exit 127
}

parse_arguments()
{
    if (($# > 1)); then
        usage >&2
        exit 2
    fi
    case "${1:---fix}" in
        --check) MODE=check ;;
        --fix) MODE=fix ;;
        --help|-h) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
}

check_environment()
{
    command -v git >/dev/null 2>&1 || fail 'git was not found in PATH'
    FORMAT_COMMAND="$(command -v "${FORMAT_COMMAND}")" ||
        fail "clang-format was not found; install it or set CLANG_FORMAT"
    [[ -f "${CONFIG_FILE}" ]] || fail "configuration was not found: ${CONFIG_FILE}"
    git rev-parse --is-inside-work-tree >/dev/null 2>&1 || fail 'the project must be a Git worktree'
    "${FORMAT_COMMAND}" --version
}

list_sources()
{
    # Include new, non-ignored sources as well as tracked ones. Keep vendor and
    # generated files outside the formatting boundary.
    git ls-files -z --cached --others --exclude-standard -- include/ src/ tests/ formal/cpp/
}

fix_sources()
{
    local HASHES INDEX RESULT FIXED=0
    local -a BEFORE AFTER

    # Compare raw contents before and after this run, not against the Git index.
    HASHES="$(git hash-object --no-filters -- "$@")"
    mapfile -t BEFORE <<<"${HASHES}"
    if "${FORMAT_COMMAND}" --style=file --fallback-style=none -i "$@"; then
        HASHES="$(git hash-object --no-filters -- "$@")"
        mapfile -t AFTER <<<"${HASHES}"
        for INDEX in "${!BEFORE[@]}"; do
            if [[ "${BEFORE[INDEX]}" != "${AFTER[INDEX]}" ]]; then
                ((FIXED += 1))
            fi
        done
        printf 'clang-format: %bfixed %d / %d file(s)%b\n' "${PASS_COLOR}" "${FIXED}" "$#" "${NO_COLOR}"
    else
        RESULT=$?
        printf 'clang-format: %bfail%b (some files may already have been formatted)\n' \
            "${FAIL_COLOR}" "${NO_COLOR}" >&2
        return "${RESULT}"
    fi
}

check_sources()
{
    local RESULT

    printf 'clang-format: check %d file(s)\n' "$#"
    if "${FORMAT_COMMAND}" --style=file --fallback-style=none --dry-run --Werror "$@"; then
        printf 'clang-format: %bpass%b\n' "${PASS_COLOR}" "${NO_COLOR}"
    else
        RESULT=$?
        printf 'clang-format: %bfail%b\n' "${FAIL_COLOR}" "${NO_COLOR}" >&2
        printf 'Run ./scripts/format_cpp.sh to apply formatting.\n' >&2
        return "${RESULT}"
    fi
}

format_sources()
{
    local FILE
    local -a FILES=()

    while IFS= read -r -d '' FILE; do
        # Do not follow symlinks or try to format tracked-but-deleted files.
        [[ -f "${FILE}" && ! -L "${FILE}" ]] || continue
        case "${FILE}" in
            *.cpp|*.cc|*.cxx|*.hpp|*.hh|*.hxx|*.h|formal/cpp/esbmc/include/numbers)
                FILES+=("${FILE}")
                ;;
        esac
    done

    if ((${#FILES[@]} == 0)); then
        printf 'clang-format: no C++ sources found\n'
        return 0
    fi
    if [[ "${MODE}" == fix ]]; then
        fix_sources "${FILES[@]}"
    else
        check_sources "${FILES[@]}"
    fi
}

main()
{
    parse_arguments "$@"
    cd -- "${REPO_DIR}"
    check_environment
    list_sources | format_sources
}

main "$@"
