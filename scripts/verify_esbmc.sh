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
    # Existing producer/caller summaries cover dense core arithmetic.
    -D ESKF_CODEGEN=0
    --quiet
)

parse_arguments()
{
    while (($#)); do
        case "$1" in
            all|quaternion|rotation|prediction|prediction32|prediction64|noise|injection|jacobian|reset|reset-generated|pcov|pcov-generated|step|solve|correction|correction-generated|step-correction|step-generated|linalg|scalar|observations|innovation|bias|magnetic|fusion|imu-policies|vertical-health|imu-inhibition|imu-prediction|gnss-quality|gnss-lifecycle|navigation-reset|navigation-recovery|height|delayed-input|delayed-views|delayed-seed|delayed-storage|delayed-history|delayed-observations|accelerometer|magnetic-runtime|ring-buffer|codegen) SUITE="$1" ;;
            --list) LIST_ONLY=1 ;;
            --shard)
                [[ "${2:-}" =~ ^([1-9][0-9]{0,2})/([1-9][0-9]{0,2})$ ]] || fail "--shard requires INDEX/COUNT, starting at 1"
                SHARD_INDEX="${BASH_REMATCH[1]}"
                SHARD_COUNT="${BASH_REMATCH[2]}"
                ((SHARD_INDEX <= SHARD_COUNT)) || fail "shard index exceeds shard count"
                shift
                ;;
            --help|-h)
                printf 'Usage: %s [all|quaternion|rotation|prediction|prediction32|prediction64|noise|injection|jacobian|reset|reset-generated|pcov|pcov-generated|step|solve|correction|correction-generated|step-correction|step-generated|linalg|scalar|observations|innovation|bias|magnetic|fusion|imu-policies|vertical-health|imu-inhibition|imu-prediction|gnss-quality|gnss-lifecycle|navigation-reset|navigation-recovery|height|delayed-input|delayed-views|delayed-seed|delayed-storage|delayed-history|delayed-observations|accelerometer|magnetic-runtime|ring-buffer|codegen] [--list] [--shard INDEX/COUNT]\n' "${0##*/}"
                printf 'Core producer/caller profiles select ESKF_CODEGEN=0 except the *-generated, delayed-views, delayed-seed, delayed-storage, delayed-history and delayed-observations suites (ESKF_CODEGEN=1); codegen checks emitted kernels directly.\n'
                printf 'Defaults to all. --list prints the exact planned profile/entry-point inventory without running proofs.\n'
                printf 'Shards are partial, disjoint inventories. Combine every shard before claiming full coverage.\n'
                printf 'injection selects caller proofs; all also runs their quaternion producer dependencies.\n'
                printf 'jacobian selects flow, entries, final predicate and boundary witnesses; all also runs its norm/scalar dependencies.\n'
                printf 'reset selects covariance-reset callers and matrix producers; all also runs their right-Jacobian dependencies.\n'
                printf 'pcov-generated selects INS generated covariance routing, actual kernel frames/mirroring and finite publication in both formats/modes/aliases; reuse pcov and its producer dependencies. Numerical accuracy remains separate; step-generated checks transaction composition.\n'
                printf 'reset-generated selects INS kernel frames/mirroring and checked callers for both reset modes and aliases; reuse jacobian/reset producers and pcov-generated publication.\n'
                printf 'correction-generated selects all five explicit model dispatches, kernel frames/mirroring, separate gain/apply callers, preparation and same-type publication; reuse correction/solve/observation producers.\n'
                printf 'step-generated selects INS prediction, injection/reset and model-specific correction transactions, plus same-type reset frames; component producers remain mandatory.\n'
                printf 'pcov selects covariance prediction and its new exact-type producers; all also runs its Exp and reset matrix dependencies.\n'
                printf 'step selects prediction, injection/reset and correction transactions; all also runs their component dependencies.\n'
                printf 'solve selects scalar producers, every Cholesky coefficient/column, orchestration and public solve/alias proofs. Root accuracy and numerical residual bounds are outside this source-level claim.\n'
                printf 'correction selects pre-injection validation, gain/Joseph routing, unpack and exact-type matrix producers. all also runs F-SOLVE and the finite-mean dependency; correction publication through injection/reset remains E-STEP.\n'
                printf 'step-correction selects full correction transactions and same-type injection/reset input frames. Mode-independent callers reuse both reset-mode producers; all also runs E-CORRECT/F-SOLVE dependencies.\n'
                printf 'linalg selects actual storage/access, block/segment, arithmetic and reduction producers for the enumerated shapes. all also runs the reused normalization, product and covariance dependencies.\n'
                printf 'scalar selects IEEE primitives, observed StandardMath dispatch and checked wrappers/aliases, plus reused wrapper/root dependencies. Pure libm summaries do not prove target library accuracy.\n'
                printf 'observations selects eight sensor models/Jacobians and checked wrappers with ESKF_MAG_TILT=0 and all state/covariance aliases. all also runs the matching E-STEP-CORRECT/E-CORRECT/F-SOLVE dependencies; physical model validity and magnetic tilt protection are not claimed.\n'
                printf 'innovation selects the actual componentwise gate for binary32/64, sizes 1/2/3 and all five vector alias partitions. No arithmetic summaries or restricted input ranges.\n'
                printf 'bias selects actual row constraints, controlled INS correction transactions and M0 sensor forwarding, with all six permissions symbolic. M1 magnetic projection is a separate obligation.\n'
                printf 'magnetic selects M1 projector/gain producers and AHRS/INS caller paths with symbolic INS bias masks, plus the reused rotation producer. E-CORRECT/F-SOLVE/injection/reset dependencies and numerical bounds remain separate.\n'
                printf 'fusion selects public covariance preparation, portable check/fuse control flow and four navigation wrappers. It reuses E-INNOV/E-CORRECT/E-STEP/E-BIAS producer contracts; replay/module wiring remains separate.\n'
                printf 'imu-policies selects actual portable fault-noise replacement, bias-noise inhibition and bias-value limits for both scalar formats and legal aliases. Detector policy, calibration and module/replay wiring are separate.\n'
                printf 'vertical-health selects actual signed ratios, reference classification, initialization/reset, clipping recurrence and inductive timestamp/hold safety. Caller wiring and root accuracy remain separate.\n'
                printf 'accelerometer selects actual magnitude history, tilt alignment, fusion conditions and AHRS/INS fusion callers with all state/P aliases; 10 ms adaptive sample history and timing compose same-type numerical contracts; module/replay wiring remains separate.\n'
                printf 'magnetic-runtime selects actual magnetic filtering, reference adoption and their inductive histories; instantaneous field plausibility and conditional bounded heading history are included; magnetic lifecycle authority is included; adapter wiring remains separate.\n'
                printf 'delayed-views selects actual 64-slot IMU/output borrowed views and scalar metadata for binary32/64 with 64-bit words. Requires reference-address tool qualification; history mutation/induction and numerical tracking remain separate.\n'
                printf 'delayed-seed checks actual 64-slot IMU construction and complete seed publication/failure/storage frames in binary32/64. Requires matching delayed-input producers; push/completion/reset remain separate.\n'
                printf 'delayed-storage checks actual 64-slot IMU ring insertion, source preservation and complete backing frames in binary32/64. Every destination partition and equality profile is required; owner admission/completion/reset remain separate.\n'
                printf 'delayed-observations selects typed storage, source control, complete owner/caller frames and context source steps; acceptance also requires Lean loop induction, reviewed source correspondence and the matching storage producers.\n'
                printf 'height selects scalar offset equations, defaults and private source admission/read-only contracts for both sources and formats; full height transaction induction and platform publication remain separate.\n'
                printf 'navigation-recovery selects actual gate diagnostic contracts, vertical limiter and both public recovery callers with every state/P alias. Same-type correction frames are reused; root accuracy and caller health/permission remain separate.\n'
                printf 'navigation-reset selects exact-type validation, symmetry-pair arithmetic and all four numerical reset callers with independent state/P aliases. Real PSD is proved separately in Lean; source permission and target backends remain separate.\n'
                printf 'ring-buffer selects actual fixed-storage operations and inductive history invariants for capacities 1/3/8 and 32/64-bit word models. Caller admission, deployed sample types, other capacities and target timing remain separate.\n'
                printf 'codegen selects memory/access safety of pinned INS prediction/reset/innovation/Joseph kernels with actual Matrix and fixed-array storage for binary32/64. No arithmetic accuracy or production integration is claimed.\n'
                printf 'gnss-lifecycle selects actual integer/Boolean lifecycle transitions and inductive permission/history safety. Estimator repair, external timestamps and actual caller scheduling remain separate.\n'
                printf 'gnss-quality selects parameter/sample domains, actual motion filters, health timers and composed public-update invariants. Caller metadata, geographic projection and accurate libm remain separate.\n'
                printf 'imu-prediction selects actual module clipping admission and noise-policy/prediction composition; E-STEP supplies the same-type terminal frame. Full module sequencing and target backend remain separate.\n'
                printf 'imu-inhibition selects actual norm/rotation/decay producers, learning permissions and inductive public-update frames. Replay/module sequencing and physical observability are separate.\n'
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

check_reference_support()
{
    local DETAILS
    if ! DETAILS="$("${ESBMC_COMMAND}" "${REPO_DIR}/tests/esbmc/reference_checks.cpp" \
        --std c++20 --64 --assign-param-nondet --quiet --unwind 70 --timeout 15s --memlimit 1g \
        --function verify_reference_identity --z3 2>&1)"; then
        printf '%s\n' "${DETAILS}" >&2
        fail 'reference-address tool preflight failed; see docs/guides/verification.md before running delayed-views, delayed-seed, delayed-storage, delayed-history, delayed-observations or all'
    fi
}

check_frame_support()
{
    local DETAILS BINARY64
    for BINARY64 in 0 1; do
        if ! DETAILS="$("${ESBMC_COMMAND}" "${REPO_DIR}/tests/esbmc/frame_checks.cpp" \
            --std c++20 --64 --assign-param-nondet --quiet --unwind 10 --timeout 15s --memlimit 1g \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --function verify_literals --z3 2>&1)"; then
            printf '%s\n' "${DETAILS}" >&2
            fail 'IEEE frame tool preflight failed; see docs/guides/verification.md'
        fi
    done
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
    local BINARY64 APPROX FUNCTION_NAME COEFFICIENT ALIAS BASE_PROFILE PREDICTION_UNWIND TAYLOR TIME_LIMIT MEMORY_LIMIT
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
                    # Pair the public rejection path with the actual private
                    # non-finite-rate producer above; retain all caller assertions.
                    verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                        -D FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT=1 --cvc5
                else
                    verify "${FUNCTION_NAME}" --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}"
                fi
            done
            # Z3 prunes the concrete boundary/overflow/translation fixtures without Bitwuzla's
            # symex memory exhaustion. The obligations are unchanged.
            verify verify_prediction_boundary_success --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" --z3
            BASE_PROFILE="${PROFILE}"
            for COEFFICIENT in 0 1 2 3 4 5; do
                PROFILE="${BASE_PROFILE}-overflow${COEFFICIENT}"
                verify verify_prediction_overflow --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                    -D "FORMAL_ESKF_PROOF_SCENARIO=${COEFFICIENT}" --z3
            done
            PROFILE="${BASE_PROFILE}"
            verify verify_ins_translation_execution --proof-unwind "${PREDICTION_UNWIND}" "${PROFILE_ARGUMENTS[@]}" --z3
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
                # Separate whole-object aliases keep pointer selection out of the FP query.
                for ALIAS in 0 1; do
                    if [[ "${APPROX}" == 1 ]]; then
                        PROFILE="${BASE_PROFILE}-actual-helper-coefficient${COEFFICIENT}-alias${ALIAS}"
                        verify verify_attitude_euler --proof-unwind "${PREDICTION_UNWIND}" \
                            --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                            -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" --cvc5
                    else
                        PROFILE="${BASE_PROFILE}-exp-summary-helper-coefficient${COEFFICIENT}-alias${ALIAS}"
                        verify verify_attitude_exp_general --proof-unwind "${PREDICTION_UNWIND}" \
                            --proof-timeout "${TIME_LIMIT}" --proof-memory "${MEMORY_LIMIT}" "${PROFILE_ARGUMENTS[@]}" \
                            -D "FORMAL_ESKF_PROOF_COEFFICIENT=${COEFFICIENT}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                            -D FORMAL_ESKF_PROOF_EXP_CONTRACT=1 --bitwuzla --multi-property
                    fi
                done
                if [[ "${APPROX}" == 1 ]]; then continue; fi
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


run_pcov_generated_suite()
{
    local BINARY64 APPROX ALIAS BASE_PROFILE
    # Scoped override: all other suites retain their dense source-proof boundary.
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    local -a PROFILE_ARGUMENTS
    for BINARY64 in 0 1; do
        BASE_PROFILE="pcov-generated-binary$((32 + 32 * BINARY64))"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        SOURCE_FILE="${PROOF_DIR}/covariance_generated.cpp"
        PROFILE="${BASE_PROFILE}-actual-kernel"
        # One query shares the emitted DAG across all mirror assertions.
        verify verify_generated_covariance --proof-unwind 226 --proof-timeout 300s "${PROFILE_ARGUMENTS[@]}"
        for ALIAS in 0 1; do
            PROFILE="${BASE_PROFILE}-actual-publisher-alias${ALIAS}"
            verify verify_publish_covariance --proof-unwind 226 --proof-timeout 300s "${PROFILE_ARGUMENTS[@]}" \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" --multi-property
        done
        SOURCE_FILE="${PROOF_DIR}/covariance_prediction.cpp"
        for APPROX in 0 1; do
            for ALIAS in 0 1; do
                PROFILE="${BASE_PROFILE}-approx${APPROX}-caller-alias${ALIAS}"
                verify verify_predict_covariance --proof-unwind 226 --proof-timeout 300s "${PROFILE_ARGUMENTS[@]}" \
                    -D FORMAL_ESKF_PROOF_INS=1 -D "ESKF_QUAT_APPROX=${APPROX}" \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_PCOV_CONTRACT=1 --multi-property
            done
        done
    done
}

run_reset_generated_suite()
{
    local BINARY64 APPROX ALIAS BASE_PROFILE
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    local -a PROFILE_ARGUMENTS
    for BINARY64 in 0 1; do
        BASE_PROFILE="reset-generated-binary$((32 + 32 * BINARY64))"
        PROFILE_ARGUMENTS=(-D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
        SOURCE_FILE="${PROOF_DIR}/covariance_generated.cpp"
        PROFILE="${BASE_PROFILE}-actual-kernel"
        verify verify_generated_reset --proof-unwind 226 --proof-timeout 300s "${PROFILE_ARGUMENTS[@]}"
        SOURCE_FILE="${PROOF_DIR}/reset.cpp"
        for APPROX in 0 1; do
            for ALIAS in 0 1; do
                PROFILE="${BASE_PROFILE}-approx${APPROX}-caller-alias${ALIAS}"
                verify verify_reset --proof-unwind 226 --proof-timeout 300s "${PROFILE_ARGUMENTS[@]}" \
                    -D FORMAL_ESKF_PROOF_INS=1 -D "ESKF_RESET_APPROX=${APPROX}" \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_RESET_CONTRACT=1 \
                    -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 --multi-property
            done
        done
    done
}

run_correction_generated_suite()
{
    local BINARY64 MODEL SIZE BOUNDARY KIND ALIAS BASE_PROFILE
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        for MODEL in 1 2 3 4 5; do
            case "${MODEL}" in
                1|3) SIZE=2 ;;
                2) SIZE=1 ;;
                4|5) SIZE=3 ;;
            esac
            BASE_PROFILE="correction-generated-binary$((32 + 32 * BINARY64))-model${MODEL}"
            ARGUMENTS=(--proof-unwind 226 --proof-timeout 300s -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
                -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}"
                -D "FORMAL_ESKF_PROOF_CORRECTION_MODEL=${MODEL}")
            SOURCE_FILE="${PROOF_DIR}/correction_generated.cpp"
            for BOUNDARY in 0 1; do
                for KIND in cross innovation joseph; do
                    PROFILE="${BASE_PROFILE}-${KIND}-boundary${BOUNDARY}"
                    verify "verify_generated_${KIND}" "${ARGUMENTS[@]}" \
                        -D "FORMAL_ESKF_PROOF_GENERATED_BOUNDARY=${BOUNDARY}"
                done
            done
            SOURCE_FILE="${PROOF_DIR}/correction.cpp"
            for KIND in gain apply; do
                PROFILE="${BASE_PROFILE}-${KIND}"
                verify "verify_generated_${KIND}" "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=1
            done
            PROFILE="${BASE_PROFILE}-preparation"
            verify verify_covariance_preparation "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=1
        done
        SOURCE_FILE="${PROOF_DIR}/correction_generated.cpp"
        for ALIAS in 0 1; do
            PROFILE="correction-generated-binary$((32 + 32 * BINARY64))-publisher-alias${ALIAS}"
            verify verify_generated_publisher --proof-unwind 226 --proof-timeout 300s \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_STATE_SIZE=15 \
                -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=3 -D FORMAL_ESKF_PROOF_CORRECTION_MODEL=5 \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" --multi-property
        done
    done
}

run_step_generated_suite()
{
    local BINARY64 MODEL SIZE APPROX ALIAS KIND BASE_PROFILE
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        BASE_PROFILE="step-generated-binary$((32 + 32 * BINARY64))"
        ARGUMENTS=(--proof-unwind 226 --proof-timeout 300s -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=3)
        SOURCE_FILE="${PROOF_DIR}/correction_generated.cpp"
        PROFILE="${BASE_PROFILE}-reset-kernel-frame"
        verify verify_generated_reset_frame "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_MODEL=5
        SOURCE_FILE="${PROOF_DIR}/step_correction_frames.cpp"
        for APPROX in 0 1; do
            PROFILE="${BASE_PROFILE}-reset${APPROX}-frame"
            verify verify_correction_reset_frame "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_STEP_FRAME=2 \
                -D "ESKF_RESET_APPROX=${APPROX}"
        done
        SOURCE_FILE="${PROOF_DIR}/step_correction.cpp"
        for MODEL in 0 1 2 3 4 5; do
            case "${MODEL}" in
                1|3) SIZE=2 ;;
                2) SIZE=1 ;;
                0|4|5) SIZE=3 ;;
            esac
            for ALIAS in 0 1 2 3; do
                PROFILE="${BASE_PROFILE}-correction-model${MODEL}-alias${ALIAS}"
                verify verify_correction_step --proof-unwind 226 --proof-timeout 300s \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_STATE_SIZE=15 \
                    -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_CORRECTION_MODEL=${MODEL}" \
                    -D FORMAL_ESKF_PROOF_BIAS_CONTROL=1 -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" \
                    -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1
            done
        done
        SOURCE_FILE="${PROOF_DIR}/step.cpp"
        for APPROX in 0 1; do
            for KIND in prediction injection_reset; do
                for ALIAS in 0 1 2 3 4 5 6 7; do
                    if [[ "${KIND}" == prediction ]] && ((ALIAS > 3)); then continue; fi
                    PROFILE="${BASE_PROFILE}-${KIND}-approx${APPROX}-alias${ALIAS}"
                    verify "verify_${KIND}_step" --proof-unwind 226 --proof-timeout 300s \
                        -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_INS=1 \
                        -D "ESKF_QUAT_APPROX=${APPROX}" -D "ESKF_RESET_APPROX=${APPROX}" \
                        -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1
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
                if ((INS)); then
                    PROFILE="${BASE_PROFILE}-quat${APPROX}-nominal-transaction"
                    verify verify_nominal_prediction_transaction --proof-unwind "${UNWIND}" "${PROFILE_ARGUMENTS[@]}" \
                        -D "ESKF_QUAT_APPROX=${APPROX}" -D FORMAL_ESKF_PROOF_STEP_FRAME=1 --multi-property
                fi
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
                SHAPES+=(15x15x6x0x3x15 15x15x0x6x15x3)
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
                for SHAPE in 3x3x1 3x1x3 3x3x3 3x3x15; do
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

run_observations_suite()
{
    # E-OBS covers unrestricted magnetic correction; default magnetic tilt
    # protection has a separate, pending obligation.
    local BINARY64 MODEL SIZE MEASUREMENT ALIAS BASE_PROFILE
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/observation_models.cpp"
        PROFILE="observations-binary$((32 + 32 * BINARY64))-rotation-producer"
        verify verify_observation_rotation --proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
            -D FORMAL_ESKF_PROOF_OBSERVATION=4 -D ESKF_MAG_TILT=0
        for MODEL in 0 1 2 3 4 5 6 7; do
            SIZE=15
            MEASUREMENT=3
            if ((MODEL == 4 || MODEL == 6)); then SIZE=3; fi
            if ((MODEL == 0 || MODEL == 2)); then MEASUREMENT=2; fi
            if ((MODEL == 1)); then MEASUREMENT=1; fi
            BASE_PROFILE="observations-binary$((32 + 32 * BINARY64))-model${MODEL}"
            ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
                -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}"
                -D "FORMAL_ESKF_PROOF_SIZE=${MEASUREMENT}" -D "FORMAL_ESKF_PROOF_OBSERVATION=${MODEL}"
                -D ESKF_MAG_TILT=0)
            if ((MODEL >= 4)); then ARGUMENTS+=(-D FORMAL_ESKF_PROOF_OBSERVATION_ROTATION=1); fi
            SOURCE_FILE="${PROOF_DIR}/observation_models.cpp"
            PROFILE="${BASE_PROFILE}-jacobian"
            verify verify_observation_jacobian "${ARGUMENTS[@]}"
            SOURCE_FILE="${PROOF_DIR}/observations.cpp"
            for ALIAS in 0 1 2 3; do
                PROFILE="${BASE_PROFILE}-caller-alias${ALIAS}"
                verify verify_observation "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT=1 \
                    -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
            done
        done
    done
}

run_magnetic_runtime_suite()
{
    local BINARY64 KIND CONTRACT MODE ENTRY ALIAS VALUES SIZE
    SOURCE_FILE="${PROOF_DIR}/magnetometer_filter.cpp"
    for BINARY64 in 0 1; do
        for KIND in initialization update; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-filter_${KIND}"
            verify "verify_magnetic_filter_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_reference.cpp"
        for KIND in trig initialization validation adoption_frame coefficients update; do
            CONTRACT=0
            [[ "${KIND}" != adoption_frame ]] || CONTRACT=1
            [[ "${KIND}" != update ]] || CONTRACT=2
            [[ "${KIND}" != coefficients ]] || CONTRACT=3
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reference_${KIND}"
            verify "verify_reference_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT=${CONTRACT}" --cvc5
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_field.cpp"
        for KIND in root angle validation geometry heading_error heading_value heading_range; do
            MODE=0
            ENTRY="verify_magnetic_${KIND}"
            case "${KIND}" in
                geometry) MODE=1 ;;
                heading_error) MODE=2 ;;
                heading_value) MODE=3 ;;
                heading_range) MODE=4; ENTRY=verify_magnetic_heading_value ;;
            esac
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-field_${KIND}"
            verify "${ENTRY}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE=${MODE}"
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_check.cpp"
        PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-forward_frame"
        verify verify_magnetic_forward_frame --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        for ALIAS in 0 1 2 3; do
            for VALUES in 0 1; do
                PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-check-alias${ALIAS}-values${VALUES}"
                verify verify_magnetic_check --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT=1 -D "FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS=${ALIAS}" \
                    -D "FORMAL_ESKF_PROOF_MAG_CHECK_VALUES=${VALUES}" --cvc5
            done
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_heading.cpp"
        for KIND in initialization norm validation acceleration; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-heading_${KIND}"
            verify "verify_heading_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --cvc5
        done
        for MODE in 0 1 2; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-heading_update-mode${MODE}"
            verify verify_heading_update --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D FORMAL_ESKF_PROOF_HEADING_CALLER=1 -D "FORMAL_ESKF_PROOF_HEADING_MODE=${MODE}" --cvc5
        done
        SOURCE_FILE="${PROOF_DIR}/magnetometer_filter.cpp"
    done
    SOURCE_FILE="${PROOF_DIR}/magnetic_reset.cpp"
    # AHRS inline numerical query; INS uses proved rectangular-product contracts.
    SIZE=3
    for BINARY64 in 0 1; do
        for ALIAS in 0 1; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_covariance${SIZE}-alias${ALIAS}"
            verify verify_magnetic_reset_covariance --proof-unwind 226 --cvc5 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_MAG_RESET_SIZE=${SIZE}" \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_reset_flow.cpp"
    for BINARY64 in 0 1; do
        for VALUES in 0 1; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_flow-values${VALUES}"
            verify verify_magnetic_reset_flow --proof-unwind 16 --cvc5 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_RESET_VALUES=${VALUES}"
        done
        for ALIAS in 0 1 2 3; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_transaction-alias${ALIAS}"
            verify verify_magnetic_reset_transaction --proof-unwind 16 --cvc5 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_RESET_PUBLIC=1 \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_reset_ins.cpp"
    for BINARY64 in 0 1; do
        for ALIAS in 0 1; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_covariance15-alias${ALIAS}"
            verify verify_magnetic_reset_ins --proof-unwind 226 --bitwuzla \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_reset_flow.cpp"
    SIZE=15
    for BINARY64 in 0 1; do
        for VALUES in 0 1; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_flow15-values${VALUES}"
            verify verify_magnetic_reset_flow --proof-unwind 226 --bitwuzla -D FORMAL_ESKF_PROOF_STATE_SIZE=15 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_RESET_VALUES=${VALUES}"
        done
        for ALIAS in 0 1 2 3; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-reset_transaction15-alias${ALIAS}"
            verify verify_magnetic_reset_transaction --proof-unwind 226 --bitwuzla -D FORMAL_ESKF_PROOF_STATE_SIZE=15 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_RESET_PUBLIC=1 \
                -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_observation.cpp"
    for BINARY64 in 0 1; do
        for CONTRACT in 0 1; do
            ENTRY=verify_magnetic_inverse_frame
            if ((CONTRACT)); then ENTRY=verify_magnetic_observation; fi
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-observation-contract${CONTRACT}"
            verify "${ENTRY}" --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT=${CONTRACT}"
        done
    done

    # Close the field/observation boundary on the backend used by correction.
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/magnetic_scalar.cpp"
        for KIND in general_root root_observer_equivalence general_angle general_geometry general_heading; do
            PROFILE="magnetic-runtime-solve-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_${KIND}" --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_check.cpp"
        PROFILE="magnetic-runtime-solve-binary$((32 + 32 * BINARY64))-forward_frame"
        verify verify_magnetic_forward_frame --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
            -D FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND=1
        for ALIAS in 0 1 2 3; do
            for VALUES in 0 1; do
                PROFILE="magnetic-runtime-solve-binary$((32 + 32 * BINARY64))-check-alias${ALIAS}-values${VALUES}"
                verify verify_magnetic_check --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND=1 -D FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT=1 \
                    -D "FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS=${ALIAS}" -D "FORMAL_ESKF_PROOF_MAG_CHECK_VALUES=${VALUES}"
            done
        done
        SOURCE_FILE="${PROOF_DIR}/magnetic_observation.cpp"
        for CONTRACT in 0 1; do
            ENTRY=verify_magnetic_inverse_frame
            if ((CONTRACT)); then ENTRY=verify_magnetic_observation; fi
            PROFILE="magnetic-runtime-solve-binary$((32 + 32 * BINARY64))-observation-contract${CONTRACT}"
            verify "${ENTRY}" --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND=1 -D "FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT=${CONTRACT}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_fusion.cpp"
    for BINARY64 in 0 1; do
        for SIZE in 3 15; do
            for ALIAS in 0 1 2 3; do
                PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-fusion${SIZE}-alias${ALIAS}"
                verify verify_magnetic_fusion --proof-unwind 226 --bitwuzla -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" -D ESKF_MAG_TILT=1
            done
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetic_model.cpp"
    for BINARY64 in 0 1; do
        for KIND in initialization update; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-model_${KIND}"
            verify "verify_model_${KIND}" --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        for ALIAS in 0 1 2; do
            PROFILE="magnetic-runtime-binary$((32 + 32 * BINARY64))-model_initialize-alias${ALIAS}"
            verify verify_model_initialize --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS=${ALIAS}"
        done
    done

    SOURCE_FILE="${PROOF_DIR}/magnetometer_lifecycle.cpp"
    for KIND in initialization update fusion reset; do
        PROFILE="magnetic-runtime-lifecycle-${KIND}"
        if [[ "${KIND}" == fusion ]]; then
            verify "verify_mag_lifecycle_${KIND}" --proof-unwind 16 --z3 --no-slice
        else
            verify "verify_mag_lifecycle_${KIND}" --proof-unwind 16 --cvc5
        fi
    done
}

run_accelerometer_suite()
{
    local BINARY64 KIND SIZE ALIAS
    SOURCE_FILE="${PROOF_DIR}/accelerometer_runtime.cpp"
    SOURCE_FILE="${PROOF_DIR}/accelerometer_sample_timing.cpp"
    PROFILE=accelerometer-double-sample_round
    verify verify_sample_round --proof-unwind 16 --bitwuzla
    SOURCE_FILE="${PROOF_DIR}/accelerometer_runtime.cpp"
    for BINARY64 in 0 1; do
        for KIND in magnitude_initialization magnitude conditions; do
            PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_accelerometer_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/accelerometer_sample.cpp"
        for KIND in initialization inverse_frame integration; do
            PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-sample_${KIND}"
            verify "verify_sample_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_SAMPLE_CONTRACT=$([[ "${KIND}" == integration ]] && printf 1 || printf 0)"
        done
        SOURCE_FILE="${PROOF_DIR}/accelerometer_sample_timing.cpp"
        PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-sample_clear"
        verify verify_sample_clear --proof-unwind 16 --bitwuzla -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        SOURCE_FILE="${PROOF_DIR}/accelerometer_sample_owner.cpp"
        PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-sample_transition"
        verify verify_sample_transition --proof-unwind 16 --bitwuzla --multi-property \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        SOURCE_FILE="${PROOF_DIR}/tilt_alignment.cpp"
        for KIND in initialization alignment; do
            PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-tilt_${KIND}"
            verify "verify_tilt_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" --cvc5
        done
        SOURCE_FILE="${PROOF_DIR}/accelerometer_fusion.cpp"
        for SIZE in 3 15; do
            for ALIAS in 0 1 2 3; do
                PROFILE="accelerometer-binary$((32 + 32 * BINARY64))-fusion${SIZE}-alias${ALIAS}"
                verify verify_accelerometer_fusion --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=3 \
                    -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}"
            done
        done
        SOURCE_FILE="${PROOF_DIR}/accelerometer_runtime.cpp"
    done
}

run_delayed_input_suite()
{
    local BINARY64
    SOURCE_FILE="${PROOF_DIR}/delayed_library.cpp"
    PROFILE=delayed-input-library-max
    verify verify_delayed_max --proof-unwind 16 --cvc5
    SOURCE_FILE="${PROOF_DIR}/delayed_input.cpp"
    PROFILE=delayed-input-timing
    verify verify_timing_configuration --proof-unwind 16 --cvc5
    for BINARY64 in 0 1; do
        PROFILE="delayed-input-binary$((32 + 32 * BINARY64))-admission"
        verify verify_imu_input --proof-unwind 5 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
    done
}

run_delayed_views_suite()
{
    local BINARY64 KIND
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    SOURCE_FILE="${PROOF_DIR}/delayed_views.cpp"
    for BINARY64 in 0 1; do
        for KIND in imu output; do
            PROFILE="delayed-views-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_${KIND}_views" --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
    done
}

run_delayed_seed_suite()
{
    local BINARY64 KIND
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    SOURCE_FILE="${PROOF_DIR}/imu_seed.cpp"
    for BINARY64 in 0 1; do
        for KIND in initialization seed; do
            PROFILE="delayed-seed-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_imu_${KIND}" --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 --array-flattener --tuple-sym-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
    done
}

run_delayed_storage_suite()
{
    local BINARY64 WRITE
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    SOURCE_FILE="${PROOF_DIR}/imu_storage.cpp"
    for BINARY64 in 0 1; do
        PROFILE="delayed-storage-binary$((32 + 32 * BINARY64))-equality"
        verify verify_imu_frame_equality --proof-unwind 5 --proof-timeout 30s --proof-memory 1g \
            --64 --z3 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_WRITE=0
        # These disjoint destination partitions exhaust the actual capacity.
        for ((WRITE = 0; WRITE < 64; ++WRITE)); do
            PROFILE="delayed-storage-binary$((32 + 32 * BINARY64))-write${WRITE}"
            verify verify_imu_storage --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_WRITE=${WRITE}"
        done
    done
}

run_delayed_history_suite()
{
    local BINARY64 SLOT KIND COMPLETION
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    SOURCE_FILE="${PROOF_DIR}/imu_producers.cpp"
    PROFILE=delayed-history-timing-frame
    verify verify_imu_timing_input_frame --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
        --64 --cvc5 -D FORMAL_ESKF_PROOF_BINARY64=0
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/imu_producers.cpp"
        for KIND in mean_rates owner_equality; do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_imu_${KIND}" --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/imu_push.cpp"
        PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-push"
        verify verify_imu_push_contract --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
            --64 --z3 --array-flattener --tuple-sym-flattener \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=0
        # Every physical slot is required; behavior is proved once per format.
        for ((SLOT = 0; SLOT < 64; ++SLOT)); do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-frame${SLOT}"
            verify verify_imu_push_frame --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=1 \
                -D "FORMAL_ESKF_PROOF_SLOT=${SLOT}"
        done
    done
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/imu_configuration.cpp"
        PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-configuration"
        verify verify_imu_configuration_contract --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
            --64 --z3 --array-flattener --tuple-sym-flattener \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=0
        for ((SLOT = 0; SLOT < 64; ++SLOT)); do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-configuration-frame${SLOT}"
            verify verify_imu_configuration_frame --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=1 \
                -D "FORMAL_ESKF_PROOF_SLOT=${SLOT}"
        done
        SOURCE_FILE="${PROOF_DIR}/imu_reset.cpp"
        for KIND in storage_reset storage_reset_owner_frame; do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_imu_${KIND}" --proof-unwind 70 --proof-timeout 240s --proof-memory 4g \
                --64 --cvc5 --array-flattener --tuple-sym-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/imu_lifecycle.cpp"
        for KIND in completion reset; do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-${KIND}"
            if [[ "${KIND}" == reset ]]; then
                verify verify_imu_reset --proof-unwind 70 --proof-timeout 240s --proof-memory 4g \
                    --64 --cvc5 --array-flattener --tuple-sym-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            else
                verify verify_imu_completion --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                    --64 --z3 --array-flattener --tuple-sym-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            fi
        done
        SOURCE_FILE="${PROOF_DIR}/imu_borrowed.cpp"
        for ((SLOT = 0; SLOT < 64; ++SLOT)); do
            PROFILE="delayed-history-binary$((32 + 32 * BINARY64))-borrowed${SLOT}"
            verify verify_imu_borrowed_input --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_SLOT=${SLOT}"
        done
    done
    # Integer-only induction depends on every source contract above and the
    # delayed-input/seed/storage/views suites; neither replaces the other.
    SOURCE_FILE="${PROOF_DIR}/imu_timeline.cpp"
    for KIND in initialization configure seed reset empty views; do
        PROFILE="delayed-history-temporal-${KIND}"
        verify "verify_imu_temporal_${KIND}" --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
            --64 --z3 --array-flattener --tuple-sym-flattener -D FORMAL_ESKF_PROOF_COMPLETION=0
    done
    for COMPLETION in 0 1; do
        if ((COMPLETION)); then KIND=completion; else KIND=push; fi
        PROFILE="delayed-history-temporal-${KIND}"
        verify verify_imu_temporal_step --proof-unwind 70 --proof-timeout 120s --proof-memory 4g \
            --64 --z3 --array-flattener --tuple-sym-flattener -D "FORMAL_ESKF_PROOF_COMPLETION=${COMPLETION}"
    done
}

# The generated include retains the original class and public methods. Only the
# ring step profiles use its additional, data-free verification entry points.
run_delayed_observations_suite()
{
    local BINARY64 SOURCE CAPACITY SLOT KIND FRAME CHUNKS CHUNK PREDICTION
    local -a FLAGS
    local -a ESBMC_ARGUMENTS=("${ESBMC_ARGUMENTS[@]/ESKF_CODEGEN=0/ESKF_CODEGEN=1}")
    local -a ORIGINAL_ARGUMENTS=("${ESBMC_ARGUMENTS[@]}")
    SOURCE_FILE="${PROOF_DIR}/observation_control.cpp"
    PROFILE=delayed-observations-double_frame_equality
    verify verify_double_frame_equality --proof-unwind 140 \
        --64 --z3 --array-flattener --tuple-sym-flattener \
        -D FORMAL_ESKF_PROOF_BINARY64=0 -D FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=0
    SOURCE_FILE="${PROOF_DIR}/observation_storage.cpp"
    for SOURCE in 0 1 2 3; do
        case "${SOURCE}" in
            0) CAPACITY=16 ;;
            1) CAPACITY=64 ;;
            2|3) CAPACITY=128 ;;
        esac
        for BINARY64 in 0 1; do
            for ((SLOT = 0; SLOT < CAPACITY; ++SLOT)); do
                PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-storage${SOURCE}-write${SLOT}"
                verify verify_observation_typed_push --proof-unwind 140 \
                    --64 --z3 --array-flattener --tuple-sym-flattener \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}" -D "FORMAL_ESKF_PROOF_WRITE=${SLOT}"
            done
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-storage${SOURCE}-reset"
            verify verify_observation_typed_reset --proof-unwind 140 \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}" -D FORMAL_ESKF_PROOF_WRITE=0
        done
    done
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/observation_control.cpp"
        for KIND in source_identity seed_source snapshot period_range observation_clock observation_configuration observation_views; do
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_${KIND}" --proof-unwind 140 \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=0
        done
        for SOURCE in 0 1 2; do
            for KIND in admission selection; do
                PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-source${SOURCE}-${KIND}"
                verify "verify_observation_${KIND}" --proof-unwind 140 \
                    --64 --z3 --array-flattener --tuple-sym-flattener \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}"
            done
        done
        SOURCE_FILE="${PROOF_DIR}/observation_frame.cpp"
        for FRAME in 0 1 2 3; do
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-frame${FRAME}"
            verify verify_observation_storage_frame --proof-unwind 140 \
                --64 --z3 --array-flattener --tuple-sym-flattener \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_FRAME=${FRAME}"
        done
    done
    ESBMC_ARGUMENTS=(-I "${REPO_DIR}/output/esbmc/ring-steps" "${ORIGINAL_ARGUMENTS[@]}")
    SOURCE_FILE="${PROOF_DIR}/observation_ring.cpp"
    for SOURCE in 0 1 2; do
        case "${SOURCE}" in
            0) CAPACITY=16 ;;
            1) CAPACITY=64 ;;
            2) CAPACITY=128 ;;
        esac
        for BINARY64 in 0 1; do
            for ((SLOT = 0; SLOT < CAPACITY; ++SLOT)); do
                for KIND in search retire; do
                    PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-source${SOURCE}-${KIND}${SLOT}"
                    verify "verify_observation_ring_${KIND}" --proof-unwind 140 \
                        --64 --z3 --array-flattener --tuple-sym-flattener \
                        -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                        -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}" -D "FORMAL_ESKF_PROOF_SLOT=${SLOT}"
                done
            done
            for KIND in finish null_output; do
                PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-source${SOURCE}-${KIND}"
                verify "verify_observation_ring_${KIND}" --proof-unwind 140 \
                    --64 --z3 --array-flattener --tuple-sym-flattener \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}" -D FORMAL_ESKF_PROOF_SLOT=0
            done
        done
    done
    ESBMC_ARGUMENTS=("${ORIGINAL_ARGUMENTS[@]}")
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/observation_owner.cpp"
        for KIND in constructor reset; do
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-owner-${KIND}"
            verify "verify_observation_${KIND}" --proof-unwind 140 --64 --z3 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        done
        SOURCE_FILE="${PROOF_DIR}/observation_context_caller.cpp"
        for PREDICTION in 0 1; do
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-context-caller${PREDICTION}"
            verify verify_context_caller --proof-unwind 140 --64 --z3 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=4 \
                -D FORMAL_ESKF_PROOF_CHUNK=0 -D "FORMAL_ESKF_PROOF_PREDICTION=${PREDICTION}"
        done
        # Each frame is split by physical storage position, not by input values.
        # Their union covers every field, including unused backing storage.
        for FRAME in 0 1 2 3; do
            case "${FRAME}" in
                0) CHUNKS=1 ;;
                1) CHUNKS=4 ;;
                2|3) CHUNKS=8 ;;
            esac
            for ((CHUNK = 0; CHUNK < CHUNKS; ++CHUNK)); do
                SOURCE_FILE="${PROOF_DIR}/observation_seed.cpp"
                PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-seed-frame${FRAME}-chunk${CHUNK}"
                verify verify_observation_seed --proof-unwind 140 --64 --z3 --fp2bv \
                    --tuple-sym-flattener --array-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_FRAME=${FRAME}" -D "FORMAL_ESKF_PROOF_CHUNK=${CHUNK}"
                SOURCE_FILE="${PROOF_DIR}/observation_record.cpp"
                PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-record-frame${FRAME}-chunk${CHUNK}"
                verify verify_context_record --proof-unwind 140 --64 --z3 --fp2bv \
                    --tuple-sym-flattener --array-flattener -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_FRAME=${FRAME}" -D "FORMAL_ESKF_PROOF_CHUNK=${CHUNK}"
                for SOURCE in 0 1 2; do
                    for KIND in enqueue pop; do
                        SOURCE_FILE="${PROOF_DIR}/observation_${KIND}.cpp"
                        PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-source${SOURCE}-${KIND}-frame${FRAME}-chunk${CHUNK}"
                        verify "verify_observation_${KIND}" --proof-unwind 140 --64 --z3 --fp2bv \
                            --tuple-sym-flattener --array-flattener \
                            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                            -D "FORMAL_ESKF_PROOF_OBSERVATION_SOURCE=${SOURCE}" \
                            -D "FORMAL_ESKF_PROOF_FRAME=${FRAME}" -D "FORMAL_ESKF_PROOF_CHUNK=${CHUNK}"
                    done
                done
            done
        done
    done
    # Only the source-step producer includes the pinned extracted lookup body.
    ESBMC_ARGUMENTS=(-I "${REPO_DIR}/output/esbmc/ring-steps/context" -I "${PROOF_DIR}" "${ORIGINAL_ARGUMENTS[@]}")
    SOURCE_FILE="${PROOF_DIR}/observation_context_producers.cpp"
    for BINARY64 in 0 1; do
        for ((SLOT = 0; SLOT < 128; ++SLOT)); do
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-context-step${SLOT}"
            verify verify_context_step --proof-unwind 140 --64 --z3 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=4 \
                -D FORMAL_ESKF_PROOF_CHUNK=0 -D "FORMAL_ESKF_PROOF_SLOT=${SLOT}"
        done
        for KIND in views empty; do
            FLAGS=(--tuple-sym-flattener --array-flattener)
            if [[ "${KIND}" == empty ]]; then FLAGS=(--fp2bv --smt-symex-guard); fi
            PROFILE="delayed-observations-binary$((32 + 32 * BINARY64))-context-${KIND}"
            verify "verify_context_${KIND}" --proof-unwind 140 --64 --z3 "${FLAGS[@]}" \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_FRAME=4 \
                -D FORMAL_ESKF_PROOF_CHUNK=0 -D FORMAL_ESKF_PROOF_SLOT=0
        done
    done
}

run_height_suite()
{
    local BINARY64 SOURCE KIND
    local -a ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/innovation_reporting.cpp"
    PROFILE=height-double-reporting
    verify verify_innovation_reporting --cvc5
    SOURCE_FILE="${PROOF_DIR}/height_library.cpp"
    PROFILE=height-library-unsigned-min
    verify verify_height_min_unsigned
    PROFILE=height-library-any-of
    verify verify_height_any_of
    for BINARY64 in 0 1; do
        SOURCE_FILE="${PROOF_DIR}/height_library.cpp"
        PROFILE="height-library-binary$((32 + 32 * BINARY64))-min"
        verify verify_height_min -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        ARGUMENTS=(--proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
            -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=1)
        SOURCE_FILE="${PROOF_DIR}/height_bias.cpp"
        for KIND in initialization prediction update; do
            PROFILE="height-bias-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_height_bias_${KIND}" "${ARGUMENTS[@]}"
        done
        SOURCE_FILE="${PROOF_DIR}/height_filter.cpp"
        for KIND in filter filter_range; do
            PROFILE="height-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_height_${KIND}" "${ARGUMENTS[@]}" --cvc5
        done
        SOURCE_FILE="${PROOF_DIR}/height_admission.cpp"
        PROFILE="height-binary$((32 + 32 * BINARY64))-initialization"
        verify verify_height_initialization "${ARGUMENTS[@]}"
        for SOURCE in 0 1; do
            SOURCE_FILE="${PROOF_DIR}/height_session_storage.cpp"
            PROFILE="height-library-binary$((32 + 32 * BINARY64))-source${SOURCE}-storage"
            verify verify_height_session_storage "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_HEIGHT_SOURCE=${SOURCE}"
            SOURCE_FILE="${PROOF_DIR}/height_admission.cpp"
            for KIND in stop admission queries; do
                PROFILE="height-binary$((32 + 32 * BINARY64))-source${SOURCE}-${KIND}"
                if [[ "${KIND}" == admission ]]; then
                    verify verify_height_admission "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_HEIGHT_SOURCE=${SOURCE}" --cvc5
                else
                    verify "verify_height_${KIND}" "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_HEIGHT_SOURCE=${SOURCE}"
                fi
            done
        done
    done
}

run_navigation_recovery_suite()
{
    local BINARY64 SIZE ALIAS KIND
    local -a ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/navigation_recovery.cpp"
    for BINARY64 in 0 1; do
        for KIND in limit_product clamp; do
            PROFILE="navigation-recovery-binary$((32 + 32 * BINARY64))-${KIND}"
            verify "verify_navigation_${KIND}" --proof-unwind 230 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=1 -D ESKF_MAG_TILT=0
        done
        for SIZE in 1 3; do
            ARGUMENTS=(--proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}" -D ESKF_MAG_TILT=0)
            PROFILE="navigation-recovery-binary$((32 + 32 * BINARY64))-size${SIZE}-diagnostics"
            verify verify_navigation_diagnostics "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY=1
            PROFILE="navigation-recovery-binary$((32 + 32 * BINARY64))-size${SIZE}-limit"
            verify verify_navigation_limit "${ARGUMENTS[@]}" --cvc5
            for ALIAS in 0 1 2 3; do
                PROFILE="navigation-recovery-binary$((32 + 32 * BINARY64))-size${SIZE}-caller-alias${ALIAS}"
                verify verify_navigation_recovery "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY=2 \
                    -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}"
            done
        done
    done
}

run_navigation_reset_suite()
{
    local BINARY64 MODEL ALIAS
    SOURCE_FILE="${PROOF_DIR}/navigation_reset.cpp"
    for BINARY64 in 0 1; do
        PROFILE="navigation-reset-binary$((32 + 32 * BINARY64))-symmetry-pair"
        verify verify_navigation_symmetry_pair --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
        for MODEL in 0 1 3; do
            PROFILE="navigation-reset-binary$((32 + 32 * BINARY64))-validation${MODEL}"
            verify verify_navigation_validation --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_NAVIGATION_RESET=${MODEL}"
        done
        for MODEL in 0 1 2 3; do
            for ALIAS in 0 1 2 3; do
                PROFILE="navigation-reset-binary$((32 + 32 * BINARY64))-model${MODEL}-alias${ALIAS}"
                verify verify_navigation_reset --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_NAVIGATION_RESET=${MODEL}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" \
                    -D FORMAL_ESKF_PROOF_NAVIGATION_CALLER=1
            done
        done
    done
    # HeightAiding uses this exact solver backend and in-place reset shape.
    for BINARY64 in 0 1; do
        PROFILE="navigation-reset-solve-binary$((32 + 32 * BINARY64))-validation"
        verify verify_navigation_validation --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
            -D FORMAL_ESKF_PROOF_NAVIGATION_RESET=1 -D FORMAL_ESKF_PROOF_NAVIGATION_SOLVE_BACKEND=1
        for MODEL in 1 2; do
            PROFILE="navigation-reset-solve-binary$((32 + 32 * BINARY64))-model${MODEL}"
            verify verify_navigation_reset --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D "FORMAL_ESKF_PROOF_NAVIGATION_RESET=${MODEL}" -D FORMAL_ESKF_PROOF_ALIAS=3 \
                -D FORMAL_ESKF_PROOF_NAVIGATION_CALLER=1 -D FORMAL_ESKF_PROOF_NAVIGATION_SOLVE_BACKEND=1
        done
    done
}

run_codegen_suite()
{
    local BINARY64 KERNEL MODEL KIND
    SOURCE_FILE="${PROOF_DIR}/codegen.cpp"
    for BINARY64 in 0 1; do
        PROFILE="codegen-binary$((32 + 32 * BINARY64))-memory"
        verify verify_codegen_memory --proof-unwind 16 --proof-timeout 90s --proof-memory 3g \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
    done
    local KERNELS=(ins_reset)
    for MODEL in horizontal_position vertical_position horizontal_velocity velocity body_vector; do
        for KIND in cross innovation joseph; do
            KERNELS+=("ins_${MODEL}_${KIND}")
        done
    done
    SOURCE_FILE="${PROOF_DIR}/codegen_products.cpp"
    for KERNEL in "${KERNELS[@]}"; do
        for BINARY64 in 0 1; do
            PROFILE="codegen-${KERNEL}-binary$((32 + 32 * BINARY64))-memory"
            verify verify_product_memory --proof-unwind 16 --proof-timeout 90s --proof-memory 3g \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "PROVE_${KERNEL^^}"
        done
    done
}

run_ring_buffer_suite()
{
    local WORD_BITS CAPACITY KIND TIME_LIMIT
    SOURCE_FILE="${PROOF_DIR}/ring_buffer.cpp"
    for WORD_BITS in 32 64; do
        for CAPACITY in 1 3 8; do
            for KIND in initialization allocate reset push selection access default_window; do
                PROFILE="ring-buffer-word${WORD_BITS}-capacity${CAPACITY}-${KIND}"
                TIME_LIMIT=180s
                if [[ "${KIND}" == selection && "${CAPACITY}" == 8 ]]; then
                    TIME_LIMIT=600s
                fi
                verify "verify_buffer_${KIND}" --proof-unwind "$((CAPACITY + 2))" --proof-timeout "${TIME_LIMIT}" --proof-memory 2g \
                    --boolector "--${WORD_BITS}" -D "FORMAL_ESKF_PROOF_CAPACITY=${CAPACITY}"
            done
        done
    done
}

run_gnss_lifecycle_suite()
{
    local KIND
    SOURCE_FILE="${PROOF_DIR}/gnss_lifecycle.cpp"
    for KIND in initialization update fusion reset_feedback vertical_feedback restart health; do
        PROFILE="gnss-lifecycle-${KIND}"
        verify "verify_gnss_lifecycle_${KIND}" --proof-unwind 16 --cvc5
    done
}

run_gnss_quality_suite()
{
    local BINARY64 KIND BOUNDARY
    SOURCE_FILE="${PROOF_DIR}/gnss_quality.cpp"
    for BINARY64 in 0 1; do
        for KIND in initialization validation health norm motion_frame motion transition; do
            BOUNDARY=0
            [[ "${KIND}" == motion_frame || "${KIND}" == motion ]] && BOUNDARY=1
            [[ "${KIND}" == transition ]] && BOUNDARY=2
            PROFILE="gnss-quality-binary$((32 + 32 * BINARY64))-${KIND}"
            if [[ "${KIND}" == transition ]]; then
                verify "verify_gnss_${KIND}" --proof-unwind 16 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_GNSS_BOUNDARY=${BOUNDARY}"
            else
                verify "verify_gnss_${KIND}" --proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_GNSS_BOUNDARY=${BOUNDARY}"
            fi
        done
    done
}

run_imu_prediction_suite()
{
    local BINARY64 KIND
    SOURCE_FILE="${PROOF_DIR}/imu_prediction.cpp"
    for BINARY64 in 0 1; do
        for KIND in admission noise bias prediction; do
            PROFILE="imu-prediction-binary$((32 + 32 * BINARY64))-${KIND}"
            case "${KIND}" in
                noise|bias) verify "verify_prediction_${KIND}" --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" ;;
                prediction) verify verify_imu_prediction --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER=1 ;;
                admission) verify verify_imu_admission --proof-unwind 230 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" ;;
            esac
        done
    done
}

run_imu_inhibition_suite()
{
    local BINARY64 KIND PROPERTY
    SOURCE_FILE="${PROOF_DIR}/imu_inhibition.cpp"
    for BINARY64 in 0 1; do
        for KIND in max_model norm rotation peak permissions inhibition; do
            PROFILE="imu-inhibition-binary$((32 + 32 * BINARY64))-${KIND}"
            if [[ "${KIND}" == peak ]]; then
                for PROPERTY in 0 1; do
                    PROFILE="imu-inhibition-binary$((32 + 32 * BINARY64))-peak${PROPERTY}"
                    verify verify_imu_peak --proof-unwind 12 --proof-timeout 300s --z3 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY=${PROPERTY}"
                done
            elif [[ "${KIND}" == inhibition ]]; then
                verify verify_imu_inhibition --proof-unwind 12 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_IMU_CALLER=1
            else
                verify "verify_imu_${KIND}" --proof-unwind 12 --z3 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            fi
        done
    done
}

run_vertical_health_suite()
{
    local BINARY64 KIND PROPERTY
    SOURCE_FILE="${PROOF_DIR}/vertical_health.cpp"
    for BINARY64 in 0 1; do
        for KIND in initialization likelihood_frame transition; do
            PROFILE="vertical-health-binary$((32 + 32 * BINARY64))-${KIND}"
            if [[ "${KIND}" == transition ]]; then
                verify verify_vertical_transition --proof-unwind 40 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_HEALTH_CALLER=1
            else
                verify "verify_vertical_${KIND}" --proof-unwind 40 --cvc5 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            fi
        done
        for PROPERTY in 0 1 2; do
            case "${PROPERTY}" in
                0) KIND=likelihood_protocol ;;
                1) KIND=likelihood_classification ;;
                2) KIND=likelihood_ratio ;;
            esac
            PROFILE="vertical-health-binary$((32 + 32 * BINARY64))-${KIND}"
            if ((PROPERTY == 2)); then
                verify verify_vertical_likelihood --proof-unwind 7 --proof-timeout 1200s --z3 --multi-property \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_HEALTH_PROPERTY=${PROPERTY}"
            elif ((PROPERTY == 1)); then
                verify verify_vertical_likelihood --proof-unwind 7 --proof-timeout 600s --bitwuzla \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_HEALTH_PROPERTY=${PROPERTY}"
            else
                verify verify_vertical_likelihood --proof-unwind 7 --bitwuzla \
                    -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_HEALTH_PROPERTY=${PROPERTY}"
            fi
        done
    done
}

run_imu_policies_suite()
{
    local BINARY64 ALIAS KIND
    local -a ARGUMENTS
    SOURCE_FILE="${PROOF_DIR}/imu_policies.cpp"
    for BINARY64 in 0 1; do
        ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_STATE_SIZE=15)
        for KIND in imu_noise bias_noise; do
            for ALIAS in 0 1; do
                PROFILE="imu-policies-binary$((32 + 32 * BINARY64))-${KIND}-alias${ALIAS}"
                verify "verify_${KIND}" "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}"
            done
        done
        PROFILE="imu-policies-binary$((32 + 32 * BINARY64))-bias_limits"
        verify verify_bias_limits "${ARGUMENTS[@]}"
    done
}

run_fusion_suite()
{
    local BINARY64 SIZE MEASUREMENT ALIAS MODEL
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        for SIZE in 3 15; do
            for MEASUREMENT in 1 2 3; do
                ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
                    -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}"
                    -D "FORMAL_ESKF_PROOF_SIZE=${MEASUREMENT}")
                SOURCE_FILE="${PROOF_DIR}/correction.cpp"
                PROFILE="fusion-binary$((32 + 32 * BINARY64))-${SIZE}x${MEASUREMENT}-preparation"
                verify verify_covariance_preparation "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_CORRECTION_CONTRACT=1
                SOURCE_FILE="${PROOF_DIR}/measurement_fusion.cpp"
                for ALIAS in 0 1 2 3; do
                    PROFILE="fusion-binary$((32 + 32 * BINARY64))-${SIZE}x${MEASUREMENT}-alias${ALIAS}"
                    verify verify_measurement_fusion "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}"
                done
            done
        done
        SOURCE_FILE="${PROOF_DIR}/navigation_fusion.cpp"
        for MODEL in 0 1 2 3; do
            case "${MODEL}" in
                0|2) MEASUREMENT=2 ;;
                1) MEASUREMENT=1 ;;
                3) MEASUREMENT=3 ;;
            esac
            for ALIAS in 0 1 2 3; do
                PROFILE="fusion-binary$((32 + 32 * BINARY64))-navigation${MODEL}-alias${ALIAS}"
                verify verify_navigation_fusion --proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${MEASUREMENT}" \
                    -D "FORMAL_ESKF_PROOF_SIZE=${MEASUREMENT}" -D "FORMAL_ESKF_PROOF_OBSERVATION=${MODEL}" \
                    -D ESKF_MAG_TILT=0 -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
            done
        done
    done
}

run_magnetic_suite()
{
    local BINARY64 SIZE ALIAS
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        # Reuse the existing producer in this standalone suite; all already
        # schedules it through observations, so do not duplicate its profile.
        if [[ "${SUITE}" == magnetic ]]; then
            SOURCE_FILE="${PROOF_DIR}/observation_models.cpp"
            PROFILE="observations-binary$((32 + 32 * BINARY64))-rotation-producer"
            verify verify_observation_rotation --proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                -D FORMAL_ESKF_PROOF_OBSERVATION=4 -D ESKF_MAG_TILT=0
        fi
        ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
            -D FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=3 -D FORMAL_ESKF_PROOF_SIZE=3 -D ESKF_MAG_TILT=1)
        SOURCE_FILE="${PROOF_DIR}/magnetic_geometry.cpp"
        PROFILE="magnetic-binary$((32 + 32 * BINARY64))-observation"
        verify verify_magnetic_observation "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_STATE_SIZE=3
        PROFILE="magnetic-binary$((32 + 32 * BINARY64))-projector"
        verify verify_magnetic_projector "${ARGUMENTS[@]}" -D FORMAL_ESKF_PROOF_STATE_SIZE=3
        for SIZE in 3 15; do
            SOURCE_FILE="${PROOF_DIR}/magnetic_geometry.cpp"
            PROFILE="magnetic-binary$((32 + 32 * BINARY64))-gain${SIZE}"
            verify verify_magnetic_gain "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}"
            SOURCE_FILE="${PROOF_DIR}/magnetic_correction.cpp"
            for ALIAS in 0 1 2 3; do
                PROFILE="magnetic-binary$((32 + 32 * BINARY64))-state${SIZE}-alias${ALIAS}"
                verify verify_magnetic_correction --proof-timeout 180s "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STATE_SIZE=${SIZE}" \
                    -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}"
            done
        done
    done
}

run_bias_suite()
{
    local BINARY64 SIZE ALIAS MODEL
    local -a ARGUMENTS
    for BINARY64 in 0 1; do
        for SIZE in 1 2 3; do
            ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
                -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}"
                -D "FORMAL_ESKF_PROOF_SIZE=${SIZE}" -D FORMAL_ESKF_PROOF_BIAS_CONTROL=1)
            SOURCE_FILE="${PROOF_DIR}/bias_gain.cpp"
            PROFILE="bias-binary$((32 + 32 * BINARY64))-gain15x${SIZE}"
            verify verify_bias_gain "${ARGUMENTS[@]}"
            SOURCE_FILE="${PROOF_DIR}/step_correction.cpp"
            for ALIAS in 0 1 2 3; do
                PROFILE="bias-binary$((32 + 32 * BINARY64))-step15x${SIZE}-alias${ALIAS}"
                verify verify_correction_step "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${ALIAS}" \
                    -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1
            done
        done
        SOURCE_FILE="${PROOF_DIR}/observations.cpp"
        for MODEL in 0 1 2 3 5 7; do
            SIZE=3
            if ((MODEL == 0 || MODEL == 2)); then SIZE=2; fi
            if ((MODEL == 1)); then SIZE=1; fi
            ARGUMENTS=(--proof-unwind 226 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}"
                -D FORMAL_ESKF_PROOF_STATE_SIZE=15 -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}"
                -D "FORMAL_ESKF_PROOF_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_OBSERVATION=${MODEL}"
                -D ESKF_MAG_TILT=0 -D FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT=1 -D FORMAL_ESKF_PROOF_BIAS_CONTROL=1)
            if ((MODEL >= 4)); then ARGUMENTS+=(-D FORMAL_ESKF_PROOF_OBSERVATION_ROTATION=1); fi
            for ALIAS in 0 1 2 3; do
                PROFILE="bias-binary$((32 + 32 * BINARY64))-observation${MODEL}-alias${ALIAS}"
                verify verify_observation "${ARGUMENTS[@]}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}"
            done
        done
    done
}

run_innovation_suite()
{
    local BINARY64 SIZE ALIAS
    SOURCE_FILE="${PROOF_DIR}/innovation.cpp"
    for BINARY64 in 0 1; do
        for SIZE in 1 2 3; do
            for ALIAS in 0 1 2 3 4; do
                PROFILE="innovation-binary$((32 + 32 * BINARY64))-size${SIZE}-alias${ALIAS}"
                verify verify_innovation --proof-unwind 6 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" \
                    -D "FORMAL_ESKF_PROOF_MEASUREMENT_SIZE=${SIZE}" -D "FORMAL_ESKF_PROOF_ALIAS=${ALIAS}" --cvc5
            done
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
        if [[ "${SUITE}" == all || "${SUITE}" == delayed-views || "${SUITE}" == delayed-seed || "${SUITE}" == delayed-storage || "${SUITE}" == delayed-history || "${SUITE}" == delayed-observations ]]; then
            check_reference_support
            check_frame_support
        fi
        if [[ "${SUITE}" == all || "${SUITE}" == delayed-observations ]]; then
            python3 "${PROOF_DIR}/ring_steps.py" "${REPO_DIR}/output/esbmc/ring-steps"
        fi
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
    if [[ "${SUITE}" == all || "${SUITE}" == reset-generated ]]; then
        run_reset_generated_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == pcov ]]; then
        run_pcov_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == pcov-generated ]]; then
        run_pcov_generated_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == step-generated ]]; then
        run_step_generated_suite
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
    if [[ "${SUITE}" == all || "${SUITE}" == correction-generated ]]; then
        run_correction_generated_suite
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
    if [[ "${SUITE}" == all || "${SUITE}" == observations ]]; then
        run_observations_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == innovation ]]; then
        run_innovation_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == bias ]]; then
        run_bias_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == magnetic ]]; then
        run_magnetic_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == fusion ]]; then
        run_fusion_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == imu-policies ]]; then
        run_imu_policies_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == vertical-health ]]; then
        run_vertical_health_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == imu-inhibition ]]; then
        run_imu_inhibition_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == imu-prediction ]]; then
        run_imu_prediction_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == gnss-quality ]]; then
        run_gnss_quality_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == codegen ]]; then
        run_codegen_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == ring-buffer ]]; then
        run_ring_buffer_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == gnss-lifecycle ]]; then
        run_gnss_lifecycle_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == navigation-reset ]]; then
        run_navigation_reset_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == navigation-recovery ]]; then
        run_navigation_recovery_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == magnetic-runtime ]]; then
        run_magnetic_runtime_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == accelerometer ]]; then
        run_accelerometer_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == height ]]; then
        run_height_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-input ]]; then
        run_delayed_input_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-views ]]; then
        run_delayed_views_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-seed ]]; then
        run_delayed_seed_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-storage ]]; then
        run_delayed_storage_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-history ]]; then
        run_delayed_history_suite
    fi
    if [[ "${SUITE}" == all || "${SUITE}" == delayed-observations ]]; then
        run_delayed_observations_suite
    fi
    if ((FAILED_CHECKS != 0)); then
        printf 'ESBMC: %d checks failed or did not complete\n' "${FAILED_CHECKS}" >&2
        return 1
    fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
