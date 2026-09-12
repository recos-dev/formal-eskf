#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROOF_DIR="${REPO_DIR}/formal/cpp/esbmc"
SOURCE_FILE="${PROOF_DIR}/quaternion.cpp"
SUITE=all
LIST_ONLY=0
SHARD_INDEX=1
SHARD_COUNT=1
PLANNED_CHECKS=0
PROFILE=quaternion-binary32
FAILED_CHECKS=0
LOCAL_ESBMC="${REPO_DIR}/tools/esbmc-8.4.0/bin/esbmc"
ESBMC_COMMAND="${FORMAL_ESKF_ESBMC:-}"
ESBMC_ARGUMENTS=(
    --std c++20
    -I "${PROOF_DIR}/include"
    -I "${REPO_DIR}/include"
    --assign-param-nondet
    --quiet
)

parse_arguments()
{
    while (($#)); do
        case "$1" in
            all|quaternion|prediction|prediction32|prediction64) SUITE="$1" ;;
            --list) LIST_ONLY=1 ;;
            --shard)
                [[ "${2:-}" =~ ^([1-9][0-9]{0,2})/([1-9][0-9]{0,2})$ ]] || fail "--shard requires INDEX/COUNT, starting at 1"
                SHARD_INDEX="${BASH_REMATCH[1]}"
                SHARD_COUNT="${BASH_REMATCH[2]}"
                ((SHARD_INDEX <= SHARD_COUNT)) || fail "shard index exceeds shard count"
                shift
                ;;
            --help|-h)
                printf 'Usage: %s [all|quaternion|prediction|prediction32|prediction64] [--list] [--shard INDEX/COUNT]\n' "${0##*/}"
                printf 'Defaults to all. --list prints the exact planned profile/entry-point inventory without running proofs.\n'
                printf 'Shards are partial, disjoint inventories. Combine every shard before claiming full coverage.\n'
                exit 0
                ;;
            *) fail "unknown argument: $1" ;;
        esac
        shift
    done
}

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
    local UNWIND_COUNT=10
    local TIME_LIMIT=120s
    local MEMORY_LIMIT=4g
    shift

    PLANNED_CHECKS=$((PLANNED_CHECKS + 1))
    if (((PLANNED_CHECKS - 1) % SHARD_COUNT + 1 != SHARD_INDEX)); then
        return
    fi

    while (($#)); do
        case "$1" in
            --proof-unwind) UNWIND_COUNT="$2" ; shift 2 ;;
            --proof-timeout) TIME_LIMIT="$2" ; shift 2 ;;
            --proof-memory) MEMORY_LIMIT="$2" ; shift 2 ;;
            *) break ;;
        esac
    done
    if ((LIST_ONLY)); then
        printf 'PLAN\t%s\t%s\t%s\n' "${SOURCE_FILE#"${REPO_DIR}/"}" "${PROFILE}" "${FUNCTION_NAME}"
        return
    fi

    local RESULT=0
    printf '==> %s [%s]\n' "${FUNCTION_NAME}" "${PROFILE}"
    "${ESBMC_COMMAND}" "${SOURCE_FILE}" "${ESBMC_ARGUMENTS[@]}" --unwind "${UNWIND_COUNT}" --timeout "${TIME_LIMIT}" --memlimit "${MEMORY_LIMIT}" \
        --function "${FUNCTION_NAME}" "$@" || RESULT=$?
    # Machine-readable execution evidence in the same log as the verifier.
    # A symbol in the traceability map is not evidence that it was executed.
    printf 'RESULT\t%s\t%s\t%s\t%d\n' "${SOURCE_FILE#"${REPO_DIR}/"}" "${PROFILE}" "${FUNCTION_NAME}" "${RESULT}"
    if ((RESULT != 0)); then
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
    fi
}

run_quaternion_suite()
{
    local ROW COLUMN

    verify verify_basic_representation
    verify verify_hamilton_product
    verify verify_construction_success
    verify verify_construction_failures
    verify verify_construction_basis
    verify verify_normalization_candidate_coefficient_0
    verify verify_normalization_candidate_coefficient_1
    verify verify_normalization_candidate_coefficient_2
    verify verify_normalization_candidate_coefficient_3
    verify verify_construction_below_threshold
    verify verify_composition --proof-unwind 5 --smt-symex-guard
    verify verify_rotation_matrix_row_0
    verify verify_rotation_matrix_row_1
    verify verify_rotation_matrix_row_2

    # A single nine-element query exhausts Bitwuzla's resource limit. Z3 proves
    # each coefficient independently; together they establish R(q) = R(-q).
    for ROW in 0 1 2; do
        for COLUMN in 0 1 2; do
            verify "verify_rotation_sign_${ROW}${COLUMN}" --default-solver z3
            verify "verify_rotate_basis_${ROW}${COLUMN}"
            verify "verify_inverse_rotate_basis_${ROW}${COLUMN}"
        done
    done

    verify verify_same_rotation_sign
    verify verify_rotate_zero --multi-property
    verify verify_hat
    verify verify_exp_zero --proof-unwind 5
    verify verify_exp_taylor_branch --proof-unwind 5
    verify verify_exp_closed_form --proof-unwind 5
    verify verify_exp_failures --proof-unwind 5
    verify verify_log_zero --proof-unwind 5
    verify verify_log_taylor_branch --proof-unwind 5
    verify verify_log_closed_form --proof-unwind 5
    verify verify_log_closed_form_scale
    verify verify_log_candidate_coefficient_0
    verify verify_log_candidate_coefficient_1
    verify verify_log_candidate_coefficient_2
    verify verify_log_principal_sign --proof-unwind 5
    verify verify_log_pi_boundary --proof-unwind 5
}

run_prediction_suite()
{
    local BINARY64 APPROX FUNCTION_NAME COEFFICIENT BASE_PROFILE ALIAS PREDICTION_UNWIND TAYLOR TIME_LIMIT MEMORY_LIMIT
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/prediction.cpp"

    for BINARY64 in 0 1; do
        [[ "${SUITE}" != prediction32 || "${BINARY64}" == 0 ]] || continue
        [[ "${SUITE}" != prediction64 || "${BINARY64}" == 1 ]] || continue
        # Fixed bounds cover storage loops and retained library models.
        # Unwinding assertions remain enabled and reject insufficient bounds.
        PREDICTION_UNWIND=$((17 + 16 * BINARY64))
        # General helper and caller queries use an explicit compositional
        # boundary. A timeout or memory failure is never treated as PASS.
        TIME_LIMIT=900s
        MEMORY_LIMIT=8g
        PROFILE="state-binary$((32 + 32 * BINARY64))"
        verify verify_state_defaults --proof-unwind 129 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_state_unpack --proof-unwind 129 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_storage_copy --proof-unwind 129 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_prediction_parameters --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        PROFILE="scalar-contract-binary$((32 + 32 * BINARY64))-actual-ieee"
        verify verify_sqrt_envelope --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_sqrt_squared_envelope --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        PROFILE="scalar-contract-binary$((32 + 32 * BINARY64))-dispatch"
        verify verify_scalar_boundary_dispatch --proof-unwind "${PREDICTION_UNWIND}" \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_SCALAR_BOUNDARY=1
        for APPROX in 0 1; do
            PROFILE="prediction-binary$((32 + 32 * BINARY64))-approx${APPROX}-standard"
            # SMT checks prune only infeasible paths. Unwinding assertions,
            # bounds/pointer checks, and IEEE arithmetic remain enabled.
            PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "ESKF_QUAT_APPROX=${APPROX}"
                --smt-symex-guard)
            for FUNCTION_NAME in verify_prediction_parameter_rejection verify_ins_non_finite_input \
                verify_attitude_non_finite verify_ahrs_zero_rate verify_ins_gyro_bias verify_attitude_norm_failure; do
                verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
            done
            BASE_PROFILE="${PROFILE}"
            for COEFFICIENT in 0 1 2 3 4 5; do
                PROFILE="${BASE_PROFILE}-overflow${COEFFICIENT}"
                verify verify_prediction_overflow --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_SCENARIO=${COEFFICIENT}"
            done
            PROFILE="${BASE_PROFILE}"
            verify verify_ins_translation_execution --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
            if [[ "${APPROX}" == 1 ]]; then
                verify verify_attitude_euler_execution --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
            else
                verify verify_attitude_exp --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
                PROFILE="${BASE_PROFILE}-taylor"
                verify verify_attitude_exp --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_TAYLOR=1
            fi
            BASE_PROFILE="prediction-binary$((32 + 32 * BINARY64))-approx${APPROX}-scalar-contract"
            # Actual helper queries use CVC5 with the checked root envelope.
            # INS callers have a different bit-vector query shape: Bitwuzla
            # avoids CVC5's memory exhaustion without changing any obligation.
            # Every default safety check and assertion remains enabled.
            PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "ESKF_QUAT_APPROX=${APPROX}"
                -D FORMAL_ESKF_PROOF_SCALAR_BOUNDARY=1)
            for COEFFICIENT in 0 1 2; do
                for ALIAS in 0 1; do
                    PROFILE="${BASE_PROFILE}-attitude-summary-translation${COEFFICIENT}-alias${ALIAS}"
                    verify verify_ins_translation --proof-unwind "${PREDICTION_UNWIND}" --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_AXIS=${COEFFICIENT}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                        -D FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT=1 --bitwuzla --multi-property
                done
            done
            for ALIAS in 0 1; do
                PROFILE="${BASE_PROFILE}-attitude-summary-ahrs-alias${ALIAS}"
                verify verify_ahrs_attitude_contract --proof-unwind "${PREDICTION_UNWIND}" \
                    --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT=1 --cvc5
            done
            # Actual helper, with NO attitude summary: all four coefficients,
            # both exhaustive Exp branches, rates in [-2,2] and arbitrary old
            # output. Every caller summary depends on these callee proofs.
            for COEFFICIENT in 0 1 2 3; do
                for TAYLOR in 0 1; do
                    [[ "${APPROX}" == 0 || "${TAYLOR}" == 0 ]] || continue
                    PROFILE="${BASE_PROFILE}-actual-helper-coefficient${COEFFICIENT}-taylor${TAYLOR}"
                    FUNCTION_NAME=verify_attitude_exp_general
                    [[ "${APPROX}" == 0 ]] || FUNCTION_NAME=verify_attitude_euler
                    verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" \
                        --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" -D "FORMAL_ESKF_PROOF_TAYLOR=${TAYLOR}" --cvc5
                done
            done
        done
    done
}

main()
{
    parse_arguments "$@"
    if ((!LIST_ONLY)); then
        select_esbmc
        check_esbmc_version
        printf 'SELECTION\t%s\tshard %d/%d\n' "${SUITE}" "${SHARD_INDEX}" "${SHARD_COUNT}"
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == quaternion ]]; then
        run_quaternion_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == prediction* ]]; then
        run_prediction_suite
    fi
    if ((FAILED_CHECKS != 0)); then
        printf 'ESBMC: %d checks failed or did not complete\n' "${FAILED_CHECKS}" >&2
        return 1
    fi
}

main "$@"
