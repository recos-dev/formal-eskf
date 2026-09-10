#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROOF_DIR="${REPO_DIR}/formal/cpp/esbmc"
SOURCE_FILE="${PROOF_DIR}/quaternion.cpp"
LOCAL_ESBMC="${REPO_DIR}/tools/esbmc-8.4.0/bin/esbmc"
ESBMC_COMMAND="${FORMAL_ESKF_ESBMC:-}"
ESBMC_ARGUMENTS=(
    "${SOURCE_FILE}"
    --std c++20
    -I "${PROOF_DIR}/include"
    -I "${REPO_DIR}/include"
    --assign-param-nondet
    --unwind 20
    --timeout 120s
    --quiet
)

fail()
{
    printf 'error: %s\n' "$*" >&2
    exit 127
}

select_esbmc()
{
    if [[ -z "${ESBMC_COMMAND}" && -x "${LOCAL_ESBMC}" ]]; then
        ESBMC_COMMAND="${LOCAL_ESBMC}"
    elif [[ -z "${ESBMC_COMMAND}" ]] && command -v esbmc >/dev/null 2>&1; then
        ESBMC_COMMAND="$(command -v esbmc)"
    fi

    if [[ -z "${ESBMC_COMMAND}" || ! -x "${ESBMC_COMMAND}" ]]; then
        fail "ESBMC was not found; install ESBMC 8.4 or set FORMAL_ESKF_ESBMC"
    fi
}

check_esbmc_version()
{
    if ! "${ESBMC_COMMAND}" --version 2>&1 | grep -q "ESBMC version 8.4.0"; then
        fail "this proof suite requires ESBMC 8.4.0"
    fi
}

verify()
{
    local FUNCTION_NAME="$1"
    shift

    printf '==> %s\n' "${FUNCTION_NAME}"
    "${ESBMC_COMMAND}" "${ESBMC_ARGUMENTS[@]}" --function "${FUNCTION_NAME}" "$@"
}

run_suite()
{
    local ROW COLUMN

    verify verify_basic_representation
    verify verify_hamilton_product
    verify verify_construction_success --no-pointer-check --no-align-check
    verify verify_construction_failures
    verify verify_rotation_matrix_row_0
    verify verify_rotation_matrix_row_1
    verify verify_rotation_matrix_row_2

    # A single nine-element query exhausts Bitwuzla's resource limit. Z3 proves
    # each coefficient independently; together they establish R(q) = R(-q).
    for ROW in 0 1 2; do
        for COLUMN in 0 1 2; do
            verify "verify_rotation_sign_${ROW}${COLUMN}" --default-solver z3
        done
    done

    verify verify_same_rotation_sign
    verify verify_hat
}

main()
{
    select_esbmc
    check_esbmc_version
    run_suite
}

main "$@"
