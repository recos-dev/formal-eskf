#!/usr/bin/env bash

set -euo pipefail

TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/verify_esbmc.sh
source "${TEST_DIR}/../scripts/verify_esbmc.sh"
RUNNER="${REPO_DIR}/scripts/verify_esbmc.sh"

check_count()
{
    local EXPECTED="$1" PATTERN="$2" ACTUAL
    ACTUAL="$(printf '%s\n' "${PLAN}" | awk -v PATTERN="${PATTERN}" '$0 ~ PATTERN { COUNT++ } END { print COUNT+0 }')"
    [[ "${ACTUAL}" == "${EXPECTED}" ]] || fail "expected ${EXPECTED} profiles matching ${PATTERN}, got ${ACTUAL}"
}

check_plan()
{
    local PLAN SHARD SHARDED='' BINARY64 APPROX COEFFICIENT ALIAS TAYLOR BASE_PROFILE
    PLAN="$("${RUNNER}" --list)"
    check_count 246 '^PLAN'
    check_count 14 'quaternion-binary32-witness'
    check_count 8 'algebra-binary32'
    check_count 8 'algebra-binary64'
    check_count 12 'normalization-binary32'
    check_count 12 'normalization-binary64'
    check_count 5 'maps-binary32'
    check_count 5 'maps-binary64'
    check_count 5 'rotation-binary32'
    check_count 5 'rotation-binary64'
    check_count 3 'noise-binary32'
    check_count 3 'noise-binary64'
    check_count 4 'injection-binary32'
    check_count 4 'injection-binary64'
    check_count 5 'jacobian-binary32'
    check_count 5 'jacobian-binary64'
    check_count 74 'prediction-binary32|state-binary32|scalar-contract-binary32'
    check_count 74 'prediction-binary64|state-binary64|scalar-contract-binary64'
    check_count 0 'shared'
    check_count 24 'attitude-summary-translation'
    check_count 8 'attitude-summary-ahrs'
    check_count 32 'approx[01].*attitude-summary'
    check_count 8 'approx1.*actual-helper-coefficient'
    check_count 8 'approx0.*exp-summary-helper-coefficient'
    check_count 16 'approx0.*actual-exp-coefficient'
    check_count 66 'approx[01]-standard'
    for BINARY64 in 0 1; do
        BASE_PROFILE="jacobian-binary$((32 + 32 * BINARY64))"
        check_count 1 "${BASE_PROFILE}-actual-finite-producer"
        check_count 1 "${BASE_PROFILE}-actual-boundary-witnesses"
        check_count 1 "${BASE_PROFILE}-all-ieee-flow"
        check_count 1 "${BASE_PROFILE}-entries-branch0"
        check_count 1 "${BASE_PROFILE}-entries-branch1"
        check_count 1 "noise-binary$((32 + 32 * BINARY64))-ahrs-all-ieee"
        check_count 1 "noise-binary$((32 + 32 * BINARY64))-ins-all-ieee"
        check_count 1 "noise-binary$((32 + 32 * BINARY64))-ins-actual-storage"
        BASE_PROFILE="normalization-binary$((32 + 32 * BINARY64))-all-ieee"
        check_count 2 "${BASE_PROFILE}-actual-producer"
        check_count 1 "${BASE_PROFILE}-actual-constructor"
        for ALIAS in 0 1; do
            check_count 1 "injection-binary$((32 + 32 * BINARY64))-ahrs-all-ieee-alias${ALIAS}"
            check_count 1 "injection-binary$((32 + 32 * BINARY64))-ins-all-ieee-alias${ALIAS}"
            check_count 1 "${BASE_PROFILE}-checked-vector-alias${ALIAS}"
            check_count 1 "${BASE_PROFILE}-quaternion-wrapper-alias${ALIAS}"
        done
        for ALIAS in 0 1 2 3 4; do
            check_count 1 "${BASE_PROFILE}-composition-alias${ALIAS}"
        done
        BASE_PROFILE="rotation-binary$((32 + 32 * BINARY64))"
        for COEFFICIENT in 0 1 2; do
            check_count 1 "${BASE_PROFILE}-actual-matrix-row${COEFFICIENT}"
        done
        check_count 1 "${BASE_PROFILE}-matrix-contract-action"
        check_count 1 "${BASE_PROFILE}-actual-matvec-basis"
        BASE_PROFILE="prediction-binary$((32 + 32 * BINARY64))"
        for APPROX in 0 1; do
            for ALIAS in 0 1; do
                check_count 1 "${BASE_PROFILE}-approx${APPROX}.*attitude-summary-ahrs-alias${ALIAS}"
                for COEFFICIENT in 0 1 2; do
                    check_count 1 "${BASE_PROFILE}-approx${APPROX}.*attitude-summary-translation${COEFFICIENT}-alias${ALIAS}"
                done
            done
        done
        for COEFFICIENT in 0 1 2 3; do
            check_count 1 "${BASE_PROFILE}-approx1.*actual-helper-coefficient${COEFFICIENT}"
            check_count 1 "${BASE_PROFILE}-approx0.*exp-summary-helper-coefficient${COEFFICIENT}"
            for TAYLOR in 0 1; do
                check_count 1 "${BASE_PROFILE}-approx0.*actual-exp-coefficient${COEFFICIENT}-taylor${TAYLOR}"
            done
        done
    done
    [[ "$(printf '%s\n' "${PLAN}" | cut -f 2,4 | sort -u | wc -l)" == 68 ]] || fail 'entry-point inventory changed'
    [[ "$(printf '%s\n' "${PLAN}" | sort -u | wc -l)" == 246 ]] || fail 'duplicate planned profile'
    for SHARD in 1 2 3 4; do
        SHARDED+="$("${RUNNER}" --list --shard "${SHARD}/4")"$'\n'
    done
    diff -u <(printf '%s\n' "${PLAN}" | sort) <(printf '%s' "${SHARDED}" | sort)
    [[ "$("${RUNNER}" quaternion --list | wc -l)" == 74 ]] || fail 'quaternion selection changed'
    [[ "$("${RUNNER}" rotation --list | wc -l)" == 10 ]] || fail 'rotation dependencies are missing'
    [[ "$("${RUNNER}" prediction --list | wc -l)" == 148 ]] || fail 'prediction selection changed'
    [[ "$("${RUNNER}" prediction32 --list | wc -l)" == 74 ]] || fail 'binary32 selection changed'
    [[ "$("${RUNNER}" prediction64 --list | wc -l)" == 74 ]] || fail 'binary64 selection changed'
    [[ "$("${RUNNER}" noise --list | wc -l)" == 6 ]] || fail 'noise configuration/dependency coverage changed'
    [[ "$("${RUNNER}" injection --list | wc -l)" == 8 ]] || fail 'injection configuration/alias coverage changed'
    [[ "$("${RUNNER}" jacobian --list | wc -l)" == 10 ]] || fail 'Jacobian flow/entries/dependency coverage changed'
}

check_jacobian_arguments()
(
    # Profile names alone cannot establish that both branch/scalar proofs run.
    verify()
    {
        local FUNCTION_NAME="$1" EXPECTED_FUNCTION=verify_right_jacobian BITS=0
        local -a EXPECTED
        shift
        [[ "${PROFILE}" != jacobian-binary64-* ]] || BITS=1
        EXPECTED=(-D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
        case "${PROFILE}" in
            *-actual-finite-producer) EXPECTED_FUNCTION=verify_right_jacobian_finite ;;
            *-actual-boundary-witnesses) EXPECTED_FUNCTION=verify_right_jacobian_boundaries ;;
            *-all-ieee-flow|*-entries-branch[01])
                EXPECTED+=(-D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1)
                if [[ "${PROFILE}" == *-entries-branch[01] ]]; then
                    EXPECTED+=(-D "FORMAL_ESKF_PROOF_JACOBIAN_BRANCH=${PROFILE##*branch}" --cvc5 --smt-symex-guard)
                fi
                ;;
            *) fail "unexpected Jacobian profile: ${PROFILE}" ;;
        esac
        [[ "${PROFILE}" == *-entries-branch[01] ]] || EXPECTED+=(--multi-property)
        [[ "${FUNCTION_NAME}" == "${EXPECTED_FUNCTION}" && "$*" == "${EXPECTED[*]}" ]] ||
            fail "wrong Jacobian function, scalar, boundary, branch or solver arguments: ${PROFILE}"
    }
    run_jacobian_suite
)

check_solver_argument_guard()
{
    local FLAG RESULT
    for FLAG in --smt-during-symex --smt-symex-guard --smt-symex-assume --smt-symex-assert --smt-thread-guard; do
        RESULT=0
        (check_solver_arguments --multi-property "${FLAG}") >/dev/null 2>&1 || RESULT=$?
        [[ "${RESULT}" == 127 ]] || fail "accepted unsupported solver combination: ${FLAG}"
        check_solver_arguments "${FLAG}"
    done
    check_solver_arguments --multi-property --cvc5
}

check_mode_constants()
(
    # Real verifier regression, not ESKF proof evidence. The two constants
    # have identical ESBMC text dumps but must produce different solver results.
    local TEST_WORK_DIR RESULT=0
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-mode-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    SOURCE_FILE="${TEST_DIR}/esbmc/mode_constants.cpp"
    run_prediction_suite()
    {
        local APPROX
        for APPROX in 0 1; do
            PROFILE="regression-approx${APPROX}"
            verify verify_mode_constants -D "ESKF_QUAT_APPROX=${APPROX}"
        done
    }
    main prediction >"${TEST_WORK_DIR}/solver.log" 2>&1 || RESULT=$?
    if [[ "${RESULT}" != 1 || "${FAILED_CHECKS}" != 1 ]] ||
        ! grep -Fxq $'RESULT\ttests/esbmc/mode_constants.cpp\tregression-approx0\tverify_mode_constants\t0' "${TEST_WORK_DIR}/solver.log" ||
        ! grep -Fxq $'RESULT\ttests/esbmc/mode_constants.cpp\tregression-approx1\tverify_mode_constants\t1' "${TEST_WORK_DIR}/solver.log" ||
        ! grep -q 'VERIFICATION SUCCESSFUL' "${TEST_WORK_DIR}/solver.log" ||
        ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/solver.log"; then
        tail -n 60 "${TEST_WORK_DIR}/solver.log" >&2
        fail 'mode-specific verification or suite failure propagation regressed'
    fi
    printf 'ESBMC solver regression: pass (mode 0 passes; mode 1 fails; suite rejects failure)\n'
)

check_rotation_contract_guards()
(
    local TEST_WORK_DIR
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-contract-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/rotation.cpp"
    PROFILE=regression-wrong-producer-contract
    verify verify_rotation_matrix -D FORMAL_ESKF_PROOF_ROTATION_CONTRACT=1 >"${TEST_WORK_DIR}/producer.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 1 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/producer.log" ||
        ! grep -q 'Runner error: verify the actual matrix producer' "${TEST_WORK_DIR}/producer.log"; then
        fail 'matrix producer accepted a substituted implementation'
    fi
    PROFILE=regression-missing-action-contract
    verify verify_rotation_action >"${TEST_WORK_DIR}/action.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 2 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/action.log" ||
        ! grep -q 'Runner error: action requires its explicit matrix summary' "${TEST_WORK_DIR}/action.log"; then
        fail 'action proof accepted a missing matrix contract'
    fi
    printf 'ESBMC contract guard tests: pass (both misconfigurations rejected)\n'
)

check_normalization_contract_guards()
(
    local TEST_WORK_DIR FUNCTION_NAME
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-normalization-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/normalization.cpp"
    for FUNCTION_NAME in verify_norm_producer verify_division_producer; do
        PROFILE="regression-wrong-${FUNCTION_NAME}-contract"
        verify "${FUNCTION_NAME}" -D FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY=1 >"${TEST_WORK_DIR}/${FUNCTION_NAME}.log" 2>&1
        if ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${FUNCTION_NAME}.log" ||
            ! grep -q 'Runner error:.*producer must be actual' "${TEST_WORK_DIR}/${FUNCTION_NAME}.log"; then
            fail "${FUNCTION_NAME} accepted a substituted implementation"
        fi
    done
    PROFILE=regression-wrong-constructor-contract
    verify verify_constructor -D FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY=2 \
        -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1 >"${TEST_WORK_DIR}/constructor.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 3 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/constructor.log" ||
        ! grep -q 'Runner error: actual quaternion constructor' "${TEST_WORK_DIR}/constructor.log"; then
        fail 'constructor producer accepted a substituted implementation'
    fi
    printf 'ESBMC normalization guard tests: pass (three substituted producers rejected)\n'
)

check_maps_contract_guards()
(
    local TEST_WORK_DIR FUNCTION_NAME
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-maps-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/maps.cpp"
    for FUNCTION_NAME in verify_squared_norm_producer verify_log_behavior; do
        PROFILE="regression-wrong-${FUNCTION_NAME}-contract"
        verify "${FUNCTION_NAME}" -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 \
            >"${TEST_WORK_DIR}/${FUNCTION_NAME}.log" 2>&1
        if ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${FUNCTION_NAME}.log" ||
            ! grep -q 'Runner error:.*actual' "${TEST_WORK_DIR}/${FUNCTION_NAME}.log"; then
            fail "${FUNCTION_NAME} accepted a substituted squared norm"
        fi
    done
    PROFILE=regression-missing-exp-contract
    verify verify_exp_behavior -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1 >"${TEST_WORK_DIR}/exp.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 3 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/exp.log" ||
        ! grep -q 'Runner error: Exp requires constructor and squared-norm boundaries' "${TEST_WORK_DIR}/exp.log"; then
        fail 'Exp accepted a missing squared-norm contract'
    fi
    printf 'ESBMC map guard tests: pass (three incompatible boundaries rejected)\n'
)

check_noise_contract_guard()
(
    local TEST_WORK_DIR
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-noise-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/process_noise.cpp"
    PROFILE=regression-wrong-noise-storage-contract
    verify verify_ins_storage --proof-unwind 145 -D FORMAL_ESKF_PROOF_STORAGE_CONTRACT=1 \
        --multi-property --multi-fail-fast 1 >"${TEST_WORK_DIR}/storage.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 1 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/storage.log" ||
        ! grep -q 'Runner error: storage producer must execute actual assignment and set_zero' "${TEST_WORK_DIR}/storage.log"; then
        fail 'noise dependency proof accepted substituted storage implementations'
    fi
    printf 'ESBMC noise guard test: pass (substituted storage producers rejected)\n'
)

check_injection_contract_guards()
(
    local TEST_WORK_DIR CONTRACT
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-injection-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/injection.cpp"
    # Check both missing callees (0/0) and an extra constructor summary (1/1).
    # Report all failures: the solver may report a caller mismatch before the
    # explicit configuration guard, so fail-fast would hide the expected label.
    for CONTRACT in 0 1; do
        PROFILE="regression-wrong-injection-contract${CONTRACT}"
        verify verify_ahrs_injection -D "FORMAL_ESKF_PROOF_INJECTION_CONTRACT=${CONTRACT}" \
            -D "FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=${CONTRACT}" --multi-property \
            >"${TEST_WORK_DIR}/boundary${CONTRACT}.log" 2>&1
        if [[ "${FAILED_CHECKS}" != "$((CONTRACT + 1))" ]] ||
            ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/boundary${CONTRACT}.log" ||
            ! grep -q 'Runner error: injection requires Exp/composition summaries and the matching constructor type' \
                "${TEST_WORK_DIR}/boundary${CONTRACT}.log"; then
            fail 'injection proof accepted a missing callee summary or incompatible constructor boundary'
        fi
    done
    printf 'ESBMC injection guard tests: pass (both incompatible boundaries rejected)\n'
)

check_jacobian_solver()
(
    local TEST_WORK_DIR FAULT
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-jacobian-solver.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${TEST_DIR}/esbmc/jacobian_checks.cpp"
    for FAULT in 0 1; do
        PROFILE="regression-jacobian-fault${FAULT}"
        verify verify_jacobian_checks -D "FORMAL_ESKF_PROOF_CHECK_FAULT=${FAULT}" \
            -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 \
            -D FORMAL_ESKF_PROOF_JACOBIAN_BRANCH=0 --cvc5 --smt-symex-guard \
            >"${TEST_WORK_DIR}/fault${FAULT}.log" 2>&1
    done
    if [[ "${FAILED_CHECKS}" != 1 ]] || ! grep -q 'VERIFICATION SUCCESSFUL' "${TEST_WORK_DIR}/fault0.log" ||
        ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/fault1.log" ||
        ! grep -q 'Regression: a changed Jacobian entry must be rejected' "${TEST_WORK_DIR}/fault1.log"; then
        tail -n 20 "${TEST_WORK_DIR}/fault0.log" >&2
        tail -n 20 "${TEST_WORK_DIR}/fault1.log" >&2
        fail 'Jacobian solver regression must accept the correct matrix and reject a changed entry'
    fi
    printf 'ESBMC Jacobian solver regression: pass (correct entries pass; changed entry fails)\n'
)

check_jacobian_contract_guards()
(
    local TEST_WORK_DIR NORM
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-jacobian-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/right_jacobian.cpp"
    for NORM in 0 1; do
        PROFILE="regression-missing-jacobian-summary${NORM}"
        verify verify_right_jacobian -D "FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=${NORM}" \
            -D "FORMAL_ESKF_PROOF_FINITE_CONTRACT=$((1 - NORM))" --multi-property \
            >"${TEST_WORK_DIR}/caller${NORM}.log" 2>&1
        if [[ "${FAILED_CHECKS}" != "$((NORM + 1))" ]] ||
            ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/caller${NORM}.log" ||
            ! grep -q 'Runner error: right Jacobian requires norm and final-predicate summaries' "${TEST_WORK_DIR}/caller${NORM}.log"; then
            fail 'Jacobian caller accepted a missing summary'
        fi
    done
    PROFILE=regression-substituted-jacobian-predicate
    verify verify_right_jacobian_finite -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 --multi-property \
        >"${TEST_WORK_DIR}/producer.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 3 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/producer.log" ||
        ! grep -q 'Runner error: matrix-finite producer must be actual' "${TEST_WORK_DIR}/producer.log"; then
        fail 'Jacobian producer accepted a substituted predicate'
    fi
    PROFILE=regression-substituted-jacobian-boundary
    verify verify_right_jacobian_boundaries -D FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT=1 --multi-property \
        >"${TEST_WORK_DIR}/boundary.log" 2>&1
    if [[ "${FAILED_CHECKS}" != 4 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/boundary.log" ||
        ! grep -q 'Runner error: right-Jacobian boundary witnesses require the actual squared norm' "${TEST_WORK_DIR}/boundary.log"; then
        fail 'Jacobian witnesses accepted a substituted norm'
    fi
    printf 'ESBMC Jacobian guard tests: pass (four incompatible boundaries rejected)\n'
)

RUN_SOLVER=0
case "${1:-}" in
    '') ;;
    --solver) RUN_SOLVER=1 ; shift ;;
    --help|-h)
        printf 'Usage: %s [--solver]\n' "${0##*/}"
        printf 'Check the profile inventory; --solver also runs a real ESBMC negative regression.\n'
        exit 0
        ;;
    *) fail "unknown argument: $1" ;;
esac
(($# == 0)) || fail 'too many arguments'
check_plan
check_jacobian_arguments
check_solver_argument_guard
printf 'ESBMC inventory tests: pass (246 profiles; 68 entry points; all shards)\n'
if ((RUN_SOLVER)); then
    check_mode_constants
    check_rotation_contract_guards
    check_normalization_contract_guards
    check_maps_contract_guards
    check_noise_contract_guard
    check_injection_contract_guards
    check_jacobian_solver
    check_jacobian_contract_guards
fi
