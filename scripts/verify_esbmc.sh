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
            all|quaternion|rotation|prediction|prediction32|prediction64|noise|injection|jacobian|reset|pcov|step|solve|correction|step-correction|linalg|scalar) SUITE="$1" ;;
            --list) LIST_ONLY=1 ;;
            --shard)
                [[ "${2:-}" =~ ^([1-9][0-9]{0,2})/([1-9][0-9]{0,2})$ ]] || fail "--shard requires INDEX/COUNT, starting at 1"
                SHARD_INDEX="${BASH_REMATCH[1]}"
                SHARD_COUNT="${BASH_REMATCH[2]}"
                ((SHARD_INDEX <= SHARD_COUNT)) || fail "shard index exceeds shard count"
                shift
                ;;
            --help|-h)
                printf 'Usage: %s [all|quaternion|rotation|prediction|prediction32|prediction64|noise|injection|jacobian|reset|pcov|step|solve|correction|step-correction|linalg|scalar] [--list] [--shard INDEX/COUNT]\n' "${0##*/}"
                printf 'Defaults to all. --list prints the exact planned profile/entry-point inventory without running proofs.\n'
                printf 'Shards are partial, disjoint inventories. Combine every shard before claiming full coverage.\n'
                printf 'injection selects caller proofs; all also runs their quaternion producer dependencies.\n'
                printf 'jacobian selects flow, entries, final predicate and boundary witnesses; all also runs its norm/scalar dependencies.\n'
                printf 'reset selects covariance-reset callers and matrix producers; all also runs their right-Jacobian dependencies.\n'
                printf 'pcov selects covariance prediction and its new exact-type producers; all also runs its Exp and reset matrix dependencies.\n'
                printf 'step selects prediction, injection/reset and correction transactions; all also runs their component dependencies.\n'
                printf 'solve selects scalar producers, every Cholesky coefficient/column, orchestration and public solve/alias proofs. Root accuracy and numerical residual bounds are outside this source-level claim.\n'
                printf 'correction selects pre-injection validation, gain/Joseph routing, unpack and exact-type matrix producers. all also runs F-SOLVE and the finite-mean dependency; correction publication through injection/reset remains E-STEP.\n'
                printf 'step-correction selects full correction transactions and same-type injection/reset input frames. Mode-independent callers reuse both reset-mode producers; all also runs E-CORRECT/F-SOLVE dependencies.\n'
                printf 'linalg selects actual storage/access, block/segment, arithmetic and reduction producers for the enumerated shapes. all also runs the reused normalization, product and covariance dependencies.\n'
                printf 'scalar selects IEEE primitives, observed StandardMath dispatch and checked wrappers/aliases, plus reused wrapper/root dependencies. Pure libm summaries do not prove target library accuracy.\n'
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

check_solver_arguments()
{
    local ARGUMENT MULTI=0 INCREMENTAL=0
    for ARGUMENT in "$@"; do
        case "${ARGUMENT}" in
            --multi-property) MULTI=1 ;;
            --smt-during-symex|--smt-symex-guard|--smt-symex-assume|--smt-symex-assert|--smt-thread-guard) INCREMENTAL=1 ;;
        esac
    done
    # ESBMC 8.4.0 can miss a known coefficient mismatch with this combination.
    # Keep incremental single-query and non-incremental multi-property separate.
    ((MULTI == 0 || INCREMENTAL == 0)) || fail 'ESBMC 8.4.0: do not combine --multi-property with incremental SMT'
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
    check_solver_arguments "${ESBMC_ARGUMENTS[@]}" "$@"
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
    # Retained success/axis/native-math witnesses, not general-domain coverage.
    SOURCE_FILE="${PROOF_DIR}/quaternion.cpp"
    PROFILE=quaternion-binary32-witness
    verify verify_construction_success
    verify verify_construction_failures
    verify verify_construction_basis
    verify verify_construction_below_threshold
    verify verify_composition --proof-unwind 5 --smt-symex-guard

    verify verify_rotate_zero --multi-property
    verify verify_exp_zero --proof-unwind 5
    verify verify_exp_taylor_branch --proof-unwind 5
    verify verify_exp_closed_form --proof-unwind 5
    verify verify_exp_failures --proof-unwind 5
    verify verify_log_zero --proof-unwind 5
    verify verify_log_taylor_branch --proof-unwind 5
    verify verify_log_closed_form --proof-unwind 5
    verify verify_log_pi_boundary --proof-unwind 5
}

run_algebra_suite()
{
    local BINARY64 ROW FUNCTION_NAME BASE_PROFILE
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/algebra.cpp"
    for BINARY64 in 0 1; do
        BASE_PROFILE="algebra-binary$((32 + 32 * BINARY64))-all-ieee"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        PROFILE="${BASE_PROFILE}"
        for FUNCTION_NAME in verify_representation verify_product verify_hat_entries verify_principal_representative; do
            verify "${FUNCTION_NAME}" "${PROFILE_ARGUMENTS[@]}" --multi-property
        done
        verify verify_rotation_comparison "${PROFILE_ARGUMENTS[@]}" --cvc5 --multi-property
        for ROW in 0 1 2; do
            PROFILE="${BASE_PROFILE}-finite-sign-row${ROW}"
            verify verify_matrix_sign "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_AXIS=${ROW}" --z3
        done
    done
}

run_normalization_suite()
{
    local BINARY64 ALIAS BASE_PROFILE FUNCTION_NAME
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/normalization.cpp"
    for BINARY64 in 0 1; do
        BASE_PROFILE="normalization-binary$((32 + 32 * BINARY64))-all-ieee"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        PROFILE="${BASE_PROFILE}-actual-producer"
        for FUNCTION_NAME in verify_norm_producer verify_division_producer; do
            verify "${FUNCTION_NAME}" "${PROFILE_ARGUMENTS[@]}" --multi-property
        done
        PROFILE="${BASE_PROFILE}-actual-constructor"
        verify verify_constructor "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY=2 --multi-property
        for ALIAS in 0 1; do
            PROFILE="${BASE_PROFILE}-checked-vector-alias${ALIAS}"
            verify verify_checked_normalization "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                -D FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY=1 --multi-property
            PROFILE="${BASE_PROFILE}-quaternion-wrapper-alias${ALIAS}"
            verify verify_normalize_wrapper "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1 --multi-property
        done
        for ALIAS in 0 1 2 3 4; do
            PROFILE="${BASE_PROFILE}-composition-alias${ALIAS}"
            verify verify_composition_general "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1 --multi-property
        done
    done
}

run_maps_suite()
{
    local BINARY64
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/maps.cpp"
    for BINARY64 in 0 1; do
        PROFILE="maps-binary$((32 + 32 * BINARY64))-all-ieee"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        verify verify_scalar_wrappers "${PROFILE_ARGUMENTS[@]}" --multi-property
        verify verify_log_arithmetic "${PROFILE_ARGUMENTS[@]}" --multi-property
        verify verify_squared_norm_producer "${PROFILE_ARGUMENTS[@]}" --multi-property
        verify verify_log_behavior "${PROFILE_ARGUMENTS[@]}" --cvc5 --multi-property
        verify verify_exp_behavior "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1 \
            -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 --cvc5 --multi-property
    done
}

run_rotation_suite()
{
    local BINARY64 ROW BASE_PROFILE
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/rotation.cpp"
    for BINARY64 in 0 1; do
        BASE_PROFILE="rotation-binary$((32 + 32 * BINARY64))"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        # The caller's opaque return is supported by all actual producer rows.
        # The basis lemma plus bounded-q finiteness retains the old basis claims.
        for ROW in 0 1 2; do
            PROFILE="${BASE_PROFILE}-actual-matrix-row${ROW}"
            verify verify_rotation_matrix "${PROFILE_ARGUMENTS[@]}" \
                -D "FORMAL_ESKF_PROOF_AXIS=${ROW}" --multi-property
        done
        PROFILE="${BASE_PROFILE}-matrix-contract-action"
        verify verify_rotation_action "${PROFILE_ARGUMENTS[@]}" \
            -D FORMAL_ESKF_PROOF_ROTATION_CONTRACT=1 --multi-property
        PROFILE="${BASE_PROFILE}-actual-matvec-basis"
        verify verify_matvec_basis "${PROFILE_ARGUMENTS[@]}" --multi-property
    done
}

run_prediction_callers()
{
    local BINARY64="$1" APPROX="$2" PREDICTION_UNWIND="$3" TIME_LIMIT="$4" MEMORY_LIMIT="$5"
    local COEFFICIENT ALIAS
    local BASE_PROFILE="prediction-binary$((32 + 32 * BINARY64))-approx${APPROX}-scalar-contract-full-domain"
    local -a PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "ESKF_QUAT_APPROX=${APPROX}"
        -D FORMAL_ESKF_PROOF_SCALAR_BOUNDARY=1)

    # Verify both modes independently. ESBMC's human-readable GOTO/symbol dumps
    # round floating constants; identical printed programs do not justify reuse.
    for COEFFICIENT in 0 1 2; do
        for ALIAS in 0 1; do
            PROFILE="${BASE_PROFILE}-attitude-summary-translation${COEFFICIENT}-alias${ALIAS}"
            verify verify_ins_translation --proof-unwind "${PREDICTION_UNWIND}" \
                --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" \
                "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_AXIS=${COEFFICIENT}" \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                -D FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT=1 --bitwuzla --multi-property
        done
    done
    for ALIAS in 0 1; do
        PROFILE="${BASE_PROFILE}-attitude-summary-ahrs-alias${ALIAS}"
        verify verify_ahrs_attitude_contract --proof-unwind "${PREDICTION_UNWIND}" \
            --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" \
            "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
            -D FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT=1 --cvc5
    done
}

run_prediction_suite()
{
    local BINARY64 APPROX FUNCTION_NAME COEFFICIENT BASE_PROFILE PREDICTION_UNWIND TAYLOR TIME_LIMIT MEMORY_LIMIT
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
        verify verify_prediction_quaternion --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --cvc5
        PROFILE="scalar-contract-binary$((32 + 32 * BINARY64))-actual-ieee"
        verify verify_sqrt_envelope --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_sqrt_squared_envelope --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        verify verify_sqrt_special_values --proof-unwind "${PREDICTION_UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
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
                verify_attitude_non_finite verify_prediction_attitude_non_finite \
                verify_ahrs_zero_rate verify_ins_gyro_bias verify_attitude_norm_failure; do
                if [[ "${FUNCTION_NAME}" == verify_prediction_attitude_non_finite ]]; then
                    # CVC5 prunes the checked-prior/non-finite-rate paths without
                    # Bitwuzla's binary64 symex timeout; the domain is unchanged.
                    verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" --cvc5
                else
                    verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
                fi
            done
            # Z3 prunes the concrete boundary fixture without Bitwuzla's
            # symex memory exhaustion. The obligations are unchanged.
            verify verify_prediction_boundary_success --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" --z3
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
            BASE_PROFILE="prediction-binary$((32 + 32 * BINARY64))-approx${APPROX}-scalar-contract-full-domain"
            run_prediction_callers "${BINARY64}" "${APPROX}" "${PREDICTION_UNWIND}" "${TIME_LIMIT}" "${MEMORY_LIMIT}"
            PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "ESKF_QUAT_APPROX=${APPROX}"
                -D FORMAL_ESKF_PROOF_SCALAR_BOUNDARY=1)
            # Full-domain helper proofs: arbitrary IEEE q/rates/output and all
            # positive finite time/threshold parameters accepted by prediction.
            # Exp and its composition caller are separate obligations. Every
            # summary depends on the matching actual-callee coefficient proofs.
            for COEFFICIENT in 0 1 2 3; do
                PROFILE="${BASE_PROFILE}-actual-helper-coefficient${COEFFICIENT}"
                if [[ "${APPROX}" == 1 ]]; then
                    verify verify_attitude_euler --proof-unwind "${PREDICTION_UNWIND}" \
                        --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" --cvc5
                    continue
                fi
                PROFILE="${BASE_PROFILE}-exp-summary-helper-coefficient${COEFFICIENT}"
                verify verify_attitude_exp_general --proof-unwind "${PREDICTION_UNWIND}" \
                    --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" -D FORMAL_ESKF_PROOF_EXP_CONTRACT=1 \
                    --bitwuzla --multi-property
                for TAYLOR in 0 1; do
                    PROFILE="${BASE_PROFILE}-actual-exp-coefficient${COEFFICIENT}-taylor${TAYLOR}"
                    verify verify_exp_general --proof-unwind "${PREDICTION_UNWIND}" \
                        --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" -D "FORMAL_ESKF_PROOF_TAYLOR=${TAYLOR}" --cvc5
                done
            done
        done
    done
}

run_noise_suite()
{
    local BINARY64 INS_TIME_LIMIT
    SOURCE_FILE="${PROOF_DIR}/process_noise.cpp"
    for BINARY64 in 0 1; do
        INS_TIME_LIMIT=180s
        if ((BINARY64)); then INS_TIME_LIMIT=300s; fi
        PROFILE="noise-binary$((32 + 32 * BINARY64))-ahrs-all-ieee"
        verify verify_ahrs_process_noise -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
        # The actual 12x12 backing store has 144 recursively stored cells.
        # Keep unwinding assertions enabled, including initialization/copy.
        PROFILE="noise-binary$((32 + 32 * BINARY64))-ins-actual-storage"
        verify verify_ins_storage --proof-unwind 145 --proof-timeout 180s \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
        PROFILE="noise-binary$((32 + 32 * BINARY64))-ins-all-ieee"
        verify verify_ins_process_noise --proof-unwind 145 --proof-timeout "${INS_TIME_LIMIT}" \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_STORAGE_CONTRACT=1 --cvc5 --multi-property
    done
}

run_injection_suite()
{
    local BINARY64 ALIAS CONFIGURATION
    SOURCE_FILE="${PROOF_DIR}/injection.cpp"
    # Existing maps/normalization/algebra profiles close these callee summaries
    # in all. Nominal injection depends on neither approximation-mode macro.
    for BINARY64 in 0 1; do
        for ALIAS in 0 1; do
            for CONFIGURATION in ahrs ins; do
                PROFILE="injection-binary$((32 + 32 * BINARY64))-${CONFIGURATION}-all-ieee-alias${ALIAS}"
                verify "verify_${CONFIGURATION}_injection" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_INJECTION_CONTRACT=1 --multi-property
            done
        done
    done
}

run_jacobian_suite()
{
    local BINARY64 BRANCH BASE_PROFILE
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/right_jacobian.cpp"
    for BINARY64 in 0 1; do
        BASE_PROFILE="jacobian-binary$((32 + 32 * BINARY64))"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        PROFILE="${BASE_PROFILE}-actual-finite-producer"
        verify verify_right_jacobian_finite "${PROFILE_ARGUMENTS[@]}" --multi-property
        PROFILE="${BASE_PROFILE}-actual-boundary-witnesses"
        verify verify_right_jacobian_boundaries "${PROFILE_ARGUMENTS[@]}" --multi-property
        PROFILE="${BASE_PROFILE}-all-ieee-flow"
        verify verify_right_jacobian "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 \
            -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 --multi-property
        # Flow proves eligibility without input restrictions. These two
        # coefficient profiles cover exactly the eligible Taylor/regular paths.
        # Guard solving prunes infeasible paths before encoding FP arithmetic.
        for BRANCH in 0 1; do
            PROFILE="${BASE_PROFILE}-entries-branch${BRANCH}"
            verify verify_right_jacobian "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 \
                -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 -D "FORMAL_ESKF_PROOF_JACOBIAN_BRANCH=${BRANCH}" \
                --cvc5 --smt-symex-guard
        done
    done
}

run_reset_suite()
{
    local BINARY64 INS ALIAS APPROX BASE_PROFILE UNWIND
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/reset.cpp"
    for BINARY64 in 0 1; do
        for INS in 0 1; do
            BASE_PROFILE="reset-binary$((32 + 32 * BINARY64))-ahrs"
            UNWIND=10
            if ((INS)); then
                BASE_PROFILE="reset-binary$((32 + 32 * BINARY64))-ins"
                # Actual initialization visits all 225 cells; keep its unwind assertion.
                UNWIND=226
            fi
            PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_INS=${INS}")
            if ((INS)); then
                PROFILE="${BASE_PROFILE}-actual-storage"
                verify verify_reset_storage --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" --multi-property
                PROFILE="${BASE_PROFILE}-actual-entry"
                verify verify_covariance_entry --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                    -D FORMAL_ESKF_PROOF_DOT_CONTRACT=1 -D FORMAL_ESKF_PROOF_ACCESS_CONTRACT=1 --multi-property
            fi
            PROFILE="${BASE_PROFILE}-actual-product"
            verify verify_covariance_product --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                -D "FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT=${INS}" --multi-property
            PROFILE="${BASE_PROFILE}-product-contract-sandwich"
            verify verify_reset_sandwich --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                -D FORMAL_ESKF_PROOF_PRODUCT_CONTRACT=1 --multi-property
            PROFILE="${BASE_PROFILE}-actual-finite"
            verify verify_reset_finite --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" --multi-property
            for ALIAS in 0 1; do
                PROFILE="${BASE_PROFILE}-actual-finalizer-alias${ALIAS}"
                verify verify_reset_finish --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 --multi-property
                for APPROX in 0 1; do
                    PROFILE="${BASE_PROFILE}-approx${APPROX}-all-ieee-alias${ALIAS}"
                    verify verify_reset --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D "ESKF_RESET_APPROX=${APPROX}" \
                        -D FORMAL_ESKF_PROOF_RESET_CONTRACT=1 -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 --multi-property
                done
            done
        done
        PROFILE="reset-binary$((32 + 32 * BINARY64))-mean-all-ieee"
        verify verify_covariance_mean -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
        PROFILE="reset-binary$((32 + 32 * BINARY64))-actual-dot"
        verify verify_covariance_dot --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
    done
    SOURCE_FILE="${PROOF_DIR}/covariance_storage.cpp"
    for BINARY64 in 0 1; do
        PROFILE="reset-binary$((32 + 32 * BINARY64))-ins-actual-vector-storage"
        verify verify_covariance_vector_storage --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
    done
}

run_pcov_suite()
{
    local BINARY64 INS ALIAS APPROX BASE_PROFILE UNWIND INS_TIME_LIMIT CALLER_TIME_LIMIT
    local -a PROFILE_ARGUMENTS
    for BINARY64 in 0 1; do
        INS_TIME_LIMIT=180s
        if ((BINARY64)); then INS_TIME_LIMIT=300s; fi
        BASE_PROFILE="pcov-binary$((32 + 32 * BINARY64))"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        SOURCE_FILE="${PROOF_DIR}/covariance_transition.cpp"
        PROFILE="${BASE_PROFILE}-actual-rotation"
        verify verify_covariance_rotation "${PROFILE_ARGUMENTS[@]}" --multi-property
        PROFILE="${BASE_PROFILE}-actual-quaternion-validation"
        verify verify_covariance_quaternion_validation "${PROFILE_ARGUMENTS[@]}" --multi-property
        for APPROX in 0 1; do
            PROFILE="${BASE_PROFILE}-approx${APPROX}-actual-transition"
            verify verify_covariance_transition "${PROFILE_ARGUMENTS[@]}" -D "ESKF_QUAT_APPROX=${APPROX}" \
                -D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=1 --multi-property
        done
        SOURCE_FILE="${PROOF_DIR}/process_noise.cpp"
        PROFILE="${BASE_PROFILE}-ahrs-actual-noise"
        verify verify_ahrs_process_noise "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_OPAQUE_MATH=1 --multi-property
        PROFILE="${BASE_PROFILE}-ins-actual-noise-storage"
        verify verify_ins_storage --proof-unwind 145 --proof-timeout 180s "${PROFILE_ARGUMENTS[@]}" \
            -D FORMAL_ESKF_PROOF_OPAQUE_MATH=1 --multi-property
        PROFILE="${BASE_PROFILE}-ins-actual-noise"
        verify verify_ins_process_noise --proof-unwind 145 --proof-timeout "${INS_TIME_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
            -D FORMAL_ESKF_PROOF_OPAQUE_MATH=1 -D FORMAL_ESKF_PROOF_STORAGE_CONTRACT=1 --cvc5 --multi-property
        SOURCE_FILE="${PROOF_DIR}/covariance_prediction.cpp"
        for INS in 0 1; do
            UNWIND=10
            CALLER_TIME_LIMIT=180s
            BASE_PROFILE="pcov-binary$((32 + 32 * BINARY64))-ahrs"
            if ((INS)); then
                UNWIND=226
                CALLER_TIME_LIMIT="${INS_TIME_LIMIT}"
                BASE_PROFILE="pcov-binary$((32 + 32 * BINARY64))-ins"
            fi
            for APPROX in 0 1; do
                for ALIAS in 0 1; do
                    PROFILE="${BASE_PROFILE}-approx${APPROX}-all-ieee-alias${ALIAS}"
                    verify verify_predict_covariance --proof-unwind "${UNWIND}" --proof-timeout "${CALLER_TIME_LIMIT}" \
                        "${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_INS=${INS}" \
                        -D "ESKF_QUAT_APPROX=${APPROX}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                        -D FORMAL_ESKF_PROOF_PCOV_CONTRACT=1 --multi-property
                done
            done
        done
    done
}

run_step_suite()
{
    local BINARY64 INS APPROX ALIAS CONFIGURATION UNWIND BASE_PROFILE
    local -a PROFILE_ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/step.cpp"
    for BINARY64 in 0 1; do
        for INS in 0 1; do
            CONFIGURATION=ahrs
            UNWIND=10
            if ((INS)); then CONFIGURATION=ins; UNWIND=226; fi
            BASE_PROFILE="step-binary$((32 + 32 * BINARY64))-${CONFIGURATION}"
            PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_INS=${INS}")
            for APPROX in 0 1; do
                PROFILE="${BASE_PROFILE}-quat${APPROX}-nominal-frame"
                verify verify_nominal_prediction_frame --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "ESKF_QUAT_APPROX=${APPROX}" -D FORMAL_ESKF_PROOF_STEP_FRAME=1 --multi-property
                # Partition alias arrangements, not coefficients or numeric
                # domains. Dynamic aggregate references mis-model/expand badly
                # in ESBMC 8.4; every legal arrangement is still mandatory.
                for ALIAS in 0 1 2 3; do
                    PROFILE="${BASE_PROFILE}-quat${APPROX}-prediction-alias${ALIAS}"
                    verify verify_prediction_step --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "ESKF_QUAT_APPROX=${APPROX}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" \
                        -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 --multi-property
                done
                for ALIAS in 0 1 2 3 4 5 6 7; do
                    PROFILE="${BASE_PROFILE}-reset${APPROX}-injection-alias${ALIAS}"
                    verify verify_injection_reset_step --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "ESKF_RESET_APPROX=${APPROX}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" \
                        -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 --multi-property
                done
            done
        done
    done
}

run_step_correction_suite()
{
    local BINARY64 SIZE MEASUREMENT ALIAS APPROX BASE_PROFILE
    local -a PROFILE_ARGUMENTS
    for BINARY64 in 0 1; do
        for SIZE in 3 15; do
            for MEASUREMENT in 1 2 3; do
                BASE_PROFILE="step-correction-binary$((32 + 32 * BINARY64))-${SIZE}x${MEASUREMENT}"
                PROFILE_ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}" \
                    -D "FORMAL_ESKF_PROOF_SIZE=${MEASUREMENT}")
                SOURCE_FILE="${PROOF_DIR}/step_correction.cpp"
                # The reset summary admits every result/status, independent of
                # the approximation macro. Prove routing/aliases once; both
                # actual reset-mode frame producers below remain mandatory.
                for ALIAS in 0 1 2 3; do
                    PROFILE="${BASE_PROFILE}-caller-alias${ALIAS}"
                    verify verify_correction_step "${PROFILE_ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1
                done
                SOURCE_FILE="${PROOF_DIR}/step_correction_frames.cpp"
                PROFILE="${BASE_PROFILE}-injection-frame"
                verify verify_correction_injection_frame "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_STEP_FRAME=1
                for APPROX in 0 1; do
                    PROFILE="${BASE_PROFILE}-reset${APPROX}-frame"
                    verify verify_correction_reset_frame "${PROFILE_ARGUMENTS[@]}" \
                        -D "ESKF_RESET_APPROX=${APPROX}" -D FORMAL_ESKF_PROOF_STEP_FRAME=2
                done
            done
        done
    done
}

run_solve_suite()
{
    local BINARY64 SIZE COLUMNS COLUMN ALIAS BASE_PROFILE
    local -a PROFILE_ARGUMENTS SHAPE_ARGUMENTS
    for BINARY64 in 0 1; do
        BASE_PROFILE="solve-binary$((32 + 32 * BINARY64))"
        SOURCE_FILE="${PROOF_DIR}/solve.cpp"
        PROFILE="${BASE_PROFILE}-arithmetic"
        verify verify_cholesky_arithmetic -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        for SIZE in 1 2 3; do
            PROFILE_ARGUMENTS=(--proof-unwind 50 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_SIZE=${SIZE}")
            SOURCE_FILE="${PROOF_DIR}/solve_coefficients.cpp"
            PROFILE="${BASE_PROFILE}-factor${SIZE}-coefficients"
            verify verify_factor_equations "${PROFILE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1
            SOURCE_FILE="${PROOF_DIR}/solve_flow.cpp"
            PROFILE="${BASE_PROFILE}-symmetry${SIZE}"
            verify verify_solve_symmetry "${PROFILE_ARGUMENTS[@]}" --multi-property
            for COLUMNS in 1 2 3 15; do
                if ((COLUMNS != SIZE && COLUMNS != 3 && COLUMNS != 15)); then continue; fi
                SHAPE_ARGUMENTS=("${PROFILE_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_COLUMNS=${COLUMNS}")
                SOURCE_FILE="${PROOF_DIR}/solve_coefficients.cpp"
                # Partition the finite index domain, not numerical inputs.
                # Every column is mandatory; no representative-column shortcut.
                for ((COLUMN=0; COLUMN<COLUMNS; ++COLUMN)); do
                    PROFILE="${BASE_PROFILE}-forward${SIZE}x${COLUMNS}-column${COLUMN}"
                    verify verify_forward_equations "${SHAPE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1 -D "FORMAL_ESKF_PROOF_COLUMN=${COLUMN}"
                    PROFILE="${BASE_PROFILE}-backward${SIZE}x${COLUMNS}-column${COLUMN}"
                    verify verify_backward_equations "${SHAPE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1 -D "FORMAL_ESKF_PROOF_COLUMN=${COLUMN}"
                done
                SOURCE_FILE="${PROOF_DIR}/solve_flow.cpp"
                for ALIAS in 0 1; do
                    if ((ALIAS == 1 && SIZE != COLUMNS)); then continue; fi
                    PROFILE="${BASE_PROFILE}-orchestration${SIZE}x${COLUMNS}-alias${ALIAS}"
                    verify verify_solve_orchestration "${SHAPE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=1 -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
                done
                for ALIAS in 0 1 2 3 4; do
                    if ((ALIAS >= 2 && SIZE != COLUMNS)); then continue; fi
                    PROFILE="${BASE_PROFILE}-left${SIZE}x${COLUMNS}-alias${ALIAS}"
                    verify verify_left_solve "${SHAPE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=2 -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
                    PROFILE="${BASE_PROFILE}-right${SIZE}x${COLUMNS}-alias${ALIAS}"
                    verify verify_right_solve "${SHAPE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=3 -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
                done
            done
        done
    done
}

run_correction_suite()
{
    local BINARY64 SIZE MEASUREMENT SHAPE ROWS INNER COLUMNS BASE_PROFILE
    local -a SCALAR_ARGUMENTS SHAPE_ARGUMENTS
    for BINARY64 in 0 1; do
        BASE_PROFILE="correction-binary$((32 + 32 * BINARY64))"
        SCALAR_ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        SOURCE_FILE="${PROOF_DIR}/correction.cpp"
        for SIZE in 3 15; do
            PROFILE="${BASE_PROFILE}-unpack${SIZE}"
            verify verify_correction_unpack "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}"
            for MEASUREMENT in 1 2 3; do
                SHAPE_ARGUMENTS=("${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" \
                    -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}" -D "FORMAL_ESKF_PROOF_SIZE=${MEASUREMENT}")
                PROFILE="${BASE_PROFILE}-validation${SIZE}x${MEASUREMENT}"
                verify verify_correction_validation "${SHAPE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=2
                PROFILE="${BASE_PROFILE}-caller${SIZE}x${MEASUREMENT}"
                verify verify_correction --proof-timeout 300s "${SHAPE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=1
            done
        done
        SOURCE_FILE="${PROOF_DIR}/correction_sandwich.cpp"
        for SHAPE in 3x1 3x2 3x3 15x1 15x2 15x3 15x15; do
            IFS=x read -r SIZE MEASUREMENT <<<"${SHAPE}"
            PROFILE="${BASE_PROFILE}-sandwich${SHAPE}"
            verify verify_correction_sandwich "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" \
                -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=1
        done
        SOURCE_FILE="${PROOF_DIR}/correction_matrix.cpp"
        PROFILE="${BASE_PROFILE}-equality"
        verify verify_correction_equality "${SCALAR_ARGUMENTS[@]}"
        PROFILE="${BASE_PROFILE}-zero-difference"
        verify verify_correction_zero_difference "${SCALAR_ARGUMENTS[@]}"
        for SIZE in 1 2 3 15; do
            PROFILE="${BASE_PROFILE}-finish${SIZE}"
            verify verify_correction_finish "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${SIZE}" \
                -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1
            PROFILE="${BASE_PROFILE}-symmetry${SIZE}"
            verify verify_correction_symmetry "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${SIZE}"
            PROFILE="${BASE_PROFILE}-dot${SIZE}"
            verify verify_correction_dot "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_INNER=${SIZE}"
            PROFILE="${BASE_PROFILE}-add${SIZE}"
            verify verify_correction_add "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${SIZE}"
            if ((SIZE == 3 || SIZE == 15)); then
                PROFILE="${BASE_PROFILE}-subtract${SIZE}"
                verify verify_correction_subtract "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${SIZE}"
            fi
        done
        for SHAPE in 1x1 2x1 3x1 15x1 1x3 2x3 3x2 3x3 1x15 2x15 3x15 15x2 15x3 2x2 15x15; do
            IFS=x read -r ROWS COLUMNS <<<"${SHAPE}"
            PROFILE="${BASE_PROFILE}-finite${SHAPE}"
            verify verify_correction_finite "${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" \
                -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
        done
        # Unique product shapes shared by innovation, delta, I-KH and both
        # Joseph terms. Prove each once, independently of sensor/config labels.
        for SHAPE in 15x15x1 15x15x15 15x15x2 15x15x3 15x1x1 15x1x15 15x2x1 15x2x15 15x2x2 15x3x1 15x3x15 15x3x3 \
            1x15x1 1x3x1 2x15x2 2x3x2 3x15x3 3x1x1 3x1x3 3x2x1 3x2x2 3x2x3 3x3x1 3x3x2 3x3x3; do
            IFS=x read -r ROWS INNER COLUMNS <<<"${SHAPE}"
            SHAPE_ARGUMENTS=("${SCALAR_ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" \
                -D "FORMAL_ESKF_PROOF_INNER=${INNER}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}")
            PROFILE="${BASE_PROFILE}-product${SHAPE}"
            verify verify_correction_product "${SHAPE_ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY=1
            PROFILE="${BASE_PROFILE}-entry${SHAPE}"
            verify verify_correction_entry --proof-timeout 300s "${SHAPE_ARGUMENTS[@]}" \
                -D FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY=2
        done
    done
}

run_linalg_suite()
{
    local BINARY64 BACKEND KIND SHAPE ROWS COLUMNS OFFSET BLOCK_ROWS BLOCK_COLUMNS START_ROW START_COLUMN OTHER_COLUMNS
    local BASE_PROFILE NAME PART
    local -a ARGUMENTS SHAPES
    SOURCE_FILE="${PROOF_DIR}/linalg.cpp"
    for BINARY64 in 0 1; do
        for BACKEND in 0 1; do
            NAME=opaque
            if ((BACKEND)); then NAME=solve; fi
            BASE_PROFILE="linalg-binary$((32 + 32 * BINARY64))-${NAME}"
            ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_LINALG_BACKEND=${BACKEND}")
            SHAPES=(1x1 2x1 3x1 4x1 15x1 1x3 2x3 3x2 3x3 1x15 2x15 3x15 15x2 15x3 2x2 15x15)
            if ((BACKEND == 0)); then SHAPES+=(12x12); fi
            for SHAPE in "${SHAPES[@]}"; do
                IFS=x read -r ROWS COLUMNS <<<"${SHAPE}"
                for KIND in storage access; do
                    if [[ "${SHAPE}" == 12x12 && "${KIND}" == storage ]]; then
                        for PART in 1 2 3; do
                            PROFILE="${BASE_PROFILE}-storage-${SHAPE}-part${PART}"
                            verify verify_linalg_storage --proof-timeout 300s "${ARGUMENTS[@]}" \
                                -D FORMAL_ESKF_PROOF_ROWS=12 -D FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=12 \
                                -D "FORMAL_ESKF_PROOF_STORAGE_PART=${PART}"
                        done
                        continue
                    fi
                    PROFILE="${BASE_PROFILE}-${KIND}-${SHAPE}"
                    verify "verify_linalg_${KIND}" --proof-timeout 300s "${ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
                done
            done
            # Elementwise vector/attitude operations. Large Joseph add/subtract
            # already have exact-type producers; unused large scale/divide
            # combinations are not added just to populate a Cartesian grid.
            for SHAPE in 1x1 2x1 3x1 4x1 3x3; do
                IFS=x read -r ROWS COLUMNS <<<"${SHAPE}"
                PROFILE="${BASE_PROFILE}-elementwise-${SHAPE}"
                verify verify_linalg_elementwise "${ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
            done
            for SHAPE in 1x3 2x3 3x1 3x2 3x3 1x15 2x15 3x15 15x1 15x2 15x3 15x15; do
                IFS=x read -r ROWS COLUMNS <<<"${SHAPE}"
                PROFILE="${BASE_PROFILE}-transpose-${SHAPE}"
                verify verify_linalg_transpose "${ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
            done
            for SHAPE in 1x1 2x2 3x3 15x15 3x1 4x1; do
                IFS=x read -r ROWS COLUMNS <<<"${SHAPE}"
                PROFILE="${BASE_PROFILE}-reductions-${SHAPE}"
                verify verify_linalg_reductions "${ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
                if ((ROWS == COLUMNS)); then
                    PROFILE="${BASE_PROFILE}-symmetry-${SHAPE}"
                    verify verify_linalg_symmetry --proof-timeout 300s "${ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}"
                fi
            done
            # Compile-time rectangles used by transition, noise, reset and
            # observation assembly; every destination coordinate is checked.
            SHAPES=(15x15x0x3x3x3 15x15x3x6x3x3 15x15x3x9x3x3 15x15x6x6x3x3 \
                15x15x6x12x3x3 15x15x3x3x3x3 15x15x9x9x3x3 15x15x12x12x3x3 \
                2x15x0x0x2x2 2x15x0x3x2x2 3x1x0x0x2x1 3x3x0x0x3x3)
            for OFFSET in 0 3 6 9 12; do
                SHAPES+=("3x15x0x${OFFSET}x3x3" "15x1x${OFFSET}x0x3x1")
            done
            if ((BACKEND == 0)); then
                for OFFSET in 0 3 6 9; do SHAPES+=("12x12x${OFFSET}x${OFFSET}x3x3"); done
            fi
            for SHAPE in "${SHAPES[@]}"; do
                IFS=x read -r ROWS COLUMNS START_ROW START_COLUMN BLOCK_ROWS BLOCK_COLUMNS <<<"${SHAPE}"
                PROFILE="${BASE_PROFILE}-block-${SHAPE}"
                verify verify_linalg_block "${ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}" \
                    -D "FORMAL_ESKF_PROOF_START_ROW=${START_ROW}" -D "FORMAL_ESKF_PROOF_START_COLUMN=${START_COLUMN}" \
                    -D "FORMAL_ESKF_PROOF_BLOCK_ROWS=${BLOCK_ROWS}" -D "FORMAL_ESKF_PROOF_BLOCK_COLUMNS=${BLOCK_COLUMNS}"
            done
            PROFILE="${BASE_PROFILE}-vector3"
            verify verify_linalg_vector "${ARGUMENTS[@]}"
            if ((BACKEND == 0)); then
                for SHAPE in 3x3x1 3x1x3; do
                    IFS=x read -r ROWS COLUMNS OTHER_COLUMNS <<<"${SHAPE}"
                    PROFILE="${BASE_PROFILE}-product-${SHAPE}"
                    verify verify_linalg_product "${ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_ROWS=${ROWS}" -D "FORMAL_ESKF_PROOF_PRODUCT_COLUMNS=${COLUMNS}" \
                        -D "FORMAL_ESKF_PROOF_OTHER_COLUMNS=${OTHER_COLUMNS}"
                done
            fi
        done
    done
}

run_scalar_suite()
{
    local BINARY64 KIND ALIAS MAX_ALIAS BASE_PROFILE
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/scalar.cpp"
        BASE_PROFILE="scalar-binary$((32 + 32 * BINARY64))"
        PROFILE="${BASE_PROFILE}-primitives"
        verify verify_scalar_primitives -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        PROFILE="${BASE_PROFILE}-backend"
        verify verify_scalar_backend -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_SCALAR_LIBM=1
        for KIND in sqrt sin_cos atan2; do
            case "${KIND}" in
                sqrt) MAX_ALIAS=1 ;;
                sin_cos) MAX_ALIAS=2 ;;
                atan2) MAX_ALIAS=3 ;;
            esac
            for ((ALIAS=0; ALIAS<=MAX_ALIAS; ++ALIAS)); do
                PROFILE="${BASE_PROFILE}-${KIND}-alias${ALIAS}"
                verify "verify_scalar_${KIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D FORMAL_ESKF_PROOF_SCALAR_LIBM=1 -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
            done
        done
        # Same source, flags and identity as maps/prediction. The standalone
        # selector includes these dependencies; all must not duplicate them.
        if [[ "${SUITE}" == scalar ]]; then
            SOURCE_FILE="${PROOF_DIR}/maps.cpp"
            PROFILE="maps-binary$((32 + 32 * BINARY64))-all-ieee"
            verify verify_scalar_wrappers -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --multi-property
            SOURCE_FILE="${PROOF_DIR}/prediction.cpp"
            PROFILE="scalar-contract-binary$((32 + 32 * BINARY64))-actual-ieee"
            for KIND in envelope squared_envelope special_values; do
                verify "verify_sqrt_${KIND}" --proof-unwind "$((17 + 16 * BINARY64))" \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            done
            PROFILE="scalar-contract-binary$((32 + 32 * BINARY64))-dispatch"
            verify verify_scalar_boundary_dispatch --proof-unwind "$((17 + 16 * BINARY64))" \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_SCALAR_BOUNDARY=1
        fi
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
        run_algebra_suite
        run_normalization_suite
        run_maps_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == quaternion || "${SUITE}" == rotation ]]; then
        run_rotation_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == prediction* ]]; then
        run_prediction_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == noise ]]; then
        run_noise_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == injection ]]; then
        run_injection_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == jacobian ]]; then
        run_jacobian_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == reset ]]; then
        run_reset_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == pcov ]]; then
        run_pcov_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == step ]]; then
        run_step_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == solve ]]; then
        run_solve_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == correction ]]; then
        run_correction_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == step || "${SUITE}" == step-correction ]]; then
        run_step_correction_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == linalg ]]; then
        run_linalg_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == scalar ]]; then
        run_scalar_suite
    fi
    if ((FAILED_CHECKS != 0)); then
        printf 'ESBMC: %d checks failed or did not complete\n' "${FAILED_CHECKS}" >&2
        return 1
    fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
