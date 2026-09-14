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
    local PLAN SHARD SHARDED='' BINARY64 APPROX COEFFICIENT ALIAS TAYLOR BASE_PROFILE CONFIGURATION SIZE COLUMNS COLUMN
    PLAN="$("${RUNNER}" --list)"
    check_count 790 '^PLAN'
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
    check_count 23 'reset-binary32'
    check_count 23 'reset-binary64'
    check_count 15 'pcov-binary32'
    check_count 15 'pcov-binary64'
    check_count 52 'step-binary32'
    check_count 52 'step-binary64'
    check_count 182 'solve-binary32'
    check_count 182 'solve-binary64'
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
        BASE_PROFILE="solve-binary$((32 + 32 * BINARY64))"
        check_count 1 "${BASE_PROFILE}-arithmetic"
        for SIZE in 1 2 3; do
            check_count 1 "${BASE_PROFILE}-factor${SIZE}-coefficients"
            check_count 1 "${BASE_PROFILE}-symmetry${SIZE}"
            for COLUMNS in 1 2 3 15; do
                if ((COLUMNS != SIZE && COLUMNS != 3 && COLUMNS != 15)); then continue; fi
                for ((COLUMN=0; COLUMN<COLUMNS; ++COLUMN)); do
                    check_count 1 "${BASE_PROFILE}-forward${SIZE}x${COLUMNS}-column${COLUMN}[[:space:]]"
                    check_count 1 "${BASE_PROFILE}-backward${SIZE}x${COLUMNS}-column${COLUMN}[[:space:]]"
                done
                check_count 1 "${BASE_PROFILE}-orchestration${SIZE}x${COLUMNS}-alias0"
                if ((SIZE == COLUMNS)); then
                    check_count 1 "${BASE_PROFILE}-orchestration${SIZE}x${COLUMNS}-alias1"
                fi
                for ALIAS in 0 1 2 3 4; do
                    if ((ALIAS >= 2 && SIZE != COLUMNS)); then continue; fi
                    check_count 1 "${BASE_PROFILE}-left${SIZE}x${COLUMNS}-alias${ALIAS}"
                    check_count 1 "${BASE_PROFILE}-right${SIZE}x${COLUMNS}-alias${ALIAS}"
                done
            done
        done
        for CONFIGURATION in ahrs ins; do
            for APPROX in 0 1; do
                BASE_PROFILE="step-binary$((32 + 32 * BINARY64))-${CONFIGURATION}"
                check_count 1 "${BASE_PROFILE}-quat${APPROX}-nominal-frame"
                for ALIAS in 0 1 2 3; do
                    check_count 1 "${BASE_PROFILE}-quat${APPROX}-prediction-alias${ALIAS}"
                done
                for ALIAS in 0 1 2 3 4 5 6 7; do
                    check_count 1 "${BASE_PROFILE}-reset${APPROX}-injection-alias${ALIAS}"
                done
            done
        done
        BASE_PROFILE="pcov-binary$((32 + 32 * BINARY64))"
        check_count 1 "${BASE_PROFILE}-actual-rotation"
        check_count 1 "${BASE_PROFILE}-actual-quaternion-validation"
        check_count 1 "${BASE_PROFILE}-ahrs-actual-noise[[:space:]]"
        check_count 1 "${BASE_PROFILE}-ins-actual-noise[[:space:]]"
        check_count 1 "${BASE_PROFILE}-ins-actual-noise-storage"
        for APPROX in 0 1; do
            check_count 1 "${BASE_PROFILE}-approx${APPROX}-actual-transition"
            for CONFIGURATION in ahrs ins; do
                for ALIAS in 0 1; do
                    check_count 1 "${BASE_PROFILE}-${CONFIGURATION}-approx${APPROX}-all-ieee-alias${ALIAS}"
                done
            done
        done
        for CONFIGURATION in ahrs ins; do
            BASE_PROFILE="reset-binary$((32 + 32 * BINARY64))-${CONFIGURATION}"
            check_count 1 "${BASE_PROFILE}-actual-product"
            check_count 1 "${BASE_PROFILE}-product-contract-sandwich"
            check_count 1 "${BASE_PROFILE}-actual-finite"
            for ALIAS in 0 1; do
                check_count 1 "${BASE_PROFILE}-actual-finalizer-alias${ALIAS}"
                for APPROX in 0 1; do
                    check_count 1 "${BASE_PROFILE}-approx${APPROX}-all-ieee-alias${ALIAS}"
                done
            done
        done
        check_count 1 "reset-binary$((32 + 32 * BINARY64))-ins-actual-storage"
        check_count 1 "reset-binary$((32 + 32 * BINARY64))-ins-actual-entry"
        check_count 1 "reset-binary$((32 + 32 * BINARY64))-ins-actual-vector-storage"
        check_count 1 "reset-binary$((32 + 32 * BINARY64))-mean-all-ieee"
        check_count 1 "reset-binary$((32 + 32 * BINARY64))-actual-dot"
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
    [[ "$(printf '%s\n' "${PLAN}" | cut -f 2,4 | sort -u | wc -l)" == 93 ]] || fail 'entry-point inventory changed'
    [[ "$(printf '%s\n' "${PLAN}" | sort -u | wc -l)" == 790 ]] || fail 'duplicate planned profile'
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
    [[ "$("${RUNNER}" reset --list | wc -l)" == 46 ]] || fail 'reset configuration/mode/alias/dependency coverage changed'
    [[ "$("${RUNNER}" pcov --list | wc -l)" == 30 ]] || fail 'covariance prediction configuration/mode/alias/dependency coverage changed'
    [[ "$("${RUNNER}" step --list | wc -l)" == 104 ]] || fail 'step configuration/mode/frame coverage changed'
    [[ "$("${RUNNER}" solve --list | wc -l)" == 364 ]] || fail 'solve scalar/column/alias coverage changed'
}

check_solve_arguments()
(
    verify()
    {
        local FUNCTION_NAME="$1" BITS SIZE KIND EXPECTED_FUNCTION EXPECTED_SOURCE
        local -a EXPECTED
        shift
        [[ "${PROFILE}" =~ ^solve-binary(32|64)-(.+)$ ]] || fail "unexpected solve profile: ${PROFILE}"
        BITS="$((BASH_REMATCH[1] / 32 - 1))"
        KIND="${BASH_REMATCH[2]}"
        EXPECTED=(-D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
        EXPECTED_SOURCE=solve_flow.cpp
        if [[ "${KIND}" == arithmetic ]]; then
            EXPECTED_SOURCE=solve.cpp
            EXPECTED_FUNCTION=verify_cholesky_arithmetic
        else
            [[ "${KIND}" =~ ^(factor|symmetry|forward|backward|orchestration|left|right)([123])(.*)$ ]] || fail "invalid solve kind: ${KIND}"
            SIZE="${BASH_REMATCH[2]}"
            EXPECTED=(--proof-unwind 50 "${EXPECTED[@]}" -D "FORMAL_ESKF_PROOF_SIZE=${SIZE}")
            if [[ "${KIND}" == factor* ]]; then
                EXPECTED_SOURCE=solve_coefficients.cpp
                EXPECTED_FUNCTION=verify_factor_equations
                EXPECTED+=(-D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1)
            elif [[ "${KIND}" == symmetry* ]]; then
                EXPECTED_FUNCTION=verify_solve_symmetry
                EXPECTED+=(--multi-property)
            elif [[ "${KIND}" =~ ^(forward|backward)[123]x([0-9]+)-column([0-9]+)$ ]]; then
                EXPECTED_SOURCE=solve_coefficients.cpp
                EXPECTED_FUNCTION="verify_${BASH_REMATCH[1]}_equations"
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_COLUMNS=${BASH_REMATCH[2]}" -D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1 -D "FORMAL_ESKF_PROOF_COLUMN=${BASH_REMATCH[3]}")
            elif [[ "${KIND}" =~ ^(orchestration|left|right)[123]x([0-9]+)-alias([0-4])$ ]]; then
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_COLUMNS=${BASH_REMATCH[2]}")
                case "${BASH_REMATCH[1]}" in
                    orchestration) EXPECTED_FUNCTION=verify_solve_orchestration; EXPECTED+=(-D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=1) ;;
                    left) EXPECTED_FUNCTION=verify_left_solve; EXPECTED+=(-D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=2) ;;
                    right) EXPECTED_FUNCTION=verify_right_solve; EXPECTED+=(-D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=3) ;;
                esac
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_ALIAS=${BASH_REMATCH[3]}")
            else
                fail "invalid solve shape/partition: ${KIND}"
            fi
        fi
        [[ "${SOURCE_FILE}" == "${PROOF_DIR}/${EXPECTED_SOURCE}" && "${FUNCTION_NAME}" == "${EXPECTED_FUNCTION}" && "$*" == "${EXPECTED[*]}" ]] ||
            fail "wrong solve source, scalar, dimensions or solver flags: ${PROFILE}"
    }
    run_solve_suite
)

check_step_arguments()
(
    verify()
    {
        local FUNCTION_NAME="$1" EXPECTED_FUNCTION BITS=0 INS=0 UNWIND=10 APPROX
        local -a EXPECTED
        shift
        [[ "${PROFILE}" != step-binary64-* ]] || BITS=1
        if [[ "${PROFILE}" == *-ins-* ]]; then INS=1; UNWIND=226; fi
        EXPECTED=(--proof-unwind "${UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BITS}" -D "FORMAL_ESKF_PROOF_INS=${INS}")
        case "${PROFILE}" in
            *-quat[01]-nominal-frame)
                EXPECTED_FUNCTION=verify_nominal_prediction_frame
                APPROX="${PROFILE#*-quat}"
                EXPECTED+=(-D "ESKF_QUAT_APPROX=${APPROX%%-*}" -D FORMAL_ESKF_PROOF_STEP_FRAME=1)
                ;;
            *-quat[01]-prediction-alias[0-3])
                EXPECTED_FUNCTION=verify_prediction_step
                APPROX="${PROFILE#*-quat}"
                EXPECTED+=(-D "ESKF_QUAT_APPROX=${APPROX%%-*}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${PROFILE##*alias}" -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1)
                ;;
            *-reset[01]-injection-alias[0-7])
                EXPECTED_FUNCTION=verify_injection_reset_step
                APPROX="${PROFILE#*-reset}"
                EXPECTED+=(-D "ESKF_RESET_APPROX=${APPROX%%-*}" -D "FORMAL_ESKF_PROOF_STEP_ALIAS=${PROFILE##*alias}" -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1)
                ;;
            *) fail "unexpected step profile: ${PROFILE}" ;;
        esac
        EXPECTED+=(--multi-property)
        [[ "${SOURCE_FILE}" == "${PROOF_DIR}/step.cpp" && "${FUNCTION_NAME}" == "${EXPECTED_FUNCTION}" && "$*" == "${EXPECTED[*]}" ]] ||
            fail "wrong step source, scalar, configuration, mode, boundary or solver arguments: ${PROFILE}"
    }
    run_step_suite
)

check_pcov_arguments()
(
    verify()
    {
        local FUNCTION_NAME="$1" BITS=0 INS=0 UNWIND=10 APPROX EXPECTED_FUNCTION TIME_LIMIT=180s
        local EXPECTED_SOURCE="${PROOF_DIR}/covariance_transition.cpp"
        local -a EXPECTED
        shift
        [[ "${PROFILE}" != pcov-binary64-* ]] || BITS=1
        EXPECTED=(-D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
        case "${PROFILE}" in
            *-actual-rotation) EXPECTED_FUNCTION=verify_covariance_rotation ;;
            *-actual-quaternion-validation) EXPECTED_FUNCTION=verify_covariance_quaternion_validation ;;
            *-approx[01]-actual-transition)
                EXPECTED_FUNCTION=verify_covariance_transition
                APPROX="${PROFILE#*-approx}"
                EXPECTED+=(-D "ESKF_QUAT_APPROX=${APPROX%%-*}" -D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=1)
                ;;
            *-ahrs-actual-noise)
                EXPECTED_FUNCTION=verify_ahrs_process_noise
                EXPECTED_SOURCE="${PROOF_DIR}/process_noise.cpp"
                EXPECTED+=(-D FORMAL_ESKF_PROOF_OPAQUE_MATH=1)
                ;;
            *-ins-actual-noise*)
                EXPECTED_SOURCE="${PROOF_DIR}/process_noise.cpp"
                EXPECTED=(--proof-unwind 145 --proof-timeout 180s "${EXPECTED[@]}" -D FORMAL_ESKF_PROOF_OPAQUE_MATH=1)
                EXPECTED_FUNCTION=verify_ins_storage
                if [[ "${PROFILE}" == *-actual-noise ]]; then
                    EXPECTED_FUNCTION=verify_ins_process_noise
                    if ((BITS)); then TIME_LIMIT=300s; fi
                    EXPECTED=(--proof-unwind 145 --proof-timeout "${TIME_LIMIT}" -D "FORMAL_ESKF_PROOF_BINARY64=${BITS}"
                        -D FORMAL_ESKF_PROOF_OPAQUE_MATH=1 -D FORMAL_ESKF_PROOF_STORAGE_CONTRACT=1 --cvc5)
                fi
                ;;
            *-all-ieee-alias[01])
                EXPECTED_SOURCE="${PROOF_DIR}/covariance_prediction.cpp"
                EXPECTED_FUNCTION=verify_predict_covariance
                if [[ "${PROFILE}" == *-ins-* ]]; then INS=1; UNWIND=226; fi
                if ((INS && BITS)); then TIME_LIMIT=300s; fi
                APPROX="${PROFILE#*-approx}"
                EXPECTED=(--proof-unwind "${UNWIND}" --proof-timeout "${TIME_LIMIT}" "${EXPECTED[@]}" -D "FORMAL_ESKF_PROOF_INS=${INS}"
                    -D "ESKF_QUAT_APPROX=${APPROX%%-*}" -D "FORMAL_ESKF_PROOF_ALIAS=${PROFILE##*alias}"
                    -D FORMAL_ESKF_PROOF_PCOV_CONTRACT=1)
                ;;
            *) fail "unexpected covariance prediction profile: ${PROFILE}" ;;
        esac
        EXPECTED+=(--multi-property)
        [[ "${SOURCE_FILE}" == "${EXPECTED_SOURCE}" && "${FUNCTION_NAME}" == "${EXPECTED_FUNCTION}" && "$*" == "${EXPECTED[*]}" ]] ||
            fail "wrong covariance prediction source, scalar, size, mode, alias, boundary or solver arguments: ${PROFILE}"
    }
    run_pcov_suite
)

check_reset_arguments()
(
    verify()
    {
        local FUNCTION_NAME="$1" EXPECTED_FUNCTION=verify_reset BITS=0 INS=0 UNWIND=10 APPROX
        local EXPECTED_SOURCE="${PROOF_DIR}/reset.cpp"
        local -a EXPECTED
        shift
        [[ "${PROFILE}" != reset-binary64-* ]] || BITS=1
        if [[ "${PROFILE}" == *-ins-* ]]; then
            INS=1
            UNWIND=226
        fi
        EXPECTED=(--proof-unwind "${UNWIND}" -D "FORMAL_ESKF_PROOF_BINARY64=${BITS}" -D "FORMAL_ESKF_PROOF_INS=${INS}")
        case "${PROFILE}" in
            *-ins-actual-vector-storage)
                EXPECTED_FUNCTION=verify_covariance_vector_storage
                EXPECTED_SOURCE="${PROOF_DIR}/covariance_storage.cpp"
                EXPECTED=(--proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
                ;;
            *-ins-actual-storage) EXPECTED_FUNCTION=verify_reset_storage ;;
            *-ins-actual-entry)
                EXPECTED_FUNCTION=verify_covariance_entry
                EXPECTED+=(-D FORMAL_ESKF_PROOF_DOT_CONTRACT=1 -D FORMAL_ESKF_PROOF_ACCESS_CONTRACT=1)
                ;;
            *-actual-product)
                EXPECTED_FUNCTION=verify_covariance_product
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT=${INS}")
                ;;
            *-product-contract-sandwich)
                EXPECTED_FUNCTION=verify_reset_sandwich
                EXPECTED+=(-D FORMAL_ESKF_PROOF_PRODUCT_CONTRACT=1)
                ;;
            *-actual-finite) EXPECTED_FUNCTION=verify_reset_finite ;;
            *-mean-all-ieee)
                EXPECTED_FUNCTION=verify_covariance_mean
                EXPECTED=(-D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
                ;;
            *-actual-dot)
                EXPECTED_FUNCTION=verify_covariance_dot
                EXPECTED=(--proof-unwind 16 -D "FORMAL_ESKF_PROOF_BINARY64=${BITS}")
                ;;
            *-actual-finalizer-alias[01])
                EXPECTED_FUNCTION=verify_reset_finish
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_ALIAS=${PROFILE##*alias}" -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1)
                ;;
            *-approx[01]-all-ieee-alias[01])
                APPROX="${PROFILE#*-approx}"
                EXPECTED+=(-D "FORMAL_ESKF_PROOF_ALIAS=${PROFILE##*alias}" -D "ESKF_RESET_APPROX=${APPROX%%-*}"
                    -D FORMAL_ESKF_PROOF_RESET_CONTRACT=1 -D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1)
                ;;
            *) fail "unexpected reset profile: ${PROFILE}" ;;
        esac
        EXPECTED+=(--multi-property)
        [[ "${SOURCE_FILE}" == "${EXPECTED_SOURCE}" && "${FUNCTION_NAME}" == "${EXPECTED_FUNCTION}" && "$*" == "${EXPECTED[*]}" ]] ||
            fail "wrong reset function, scalar, size, alias, mode, boundary or solver arguments: ${PROFILE}"
    }
    run_reset_suite
)

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

check_reset_contract_guards()
(
    local TEST_WORK_DIR FUNCTION_NAME FLAGS EXPECTED RESULT_COUNT=0
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-reset-test.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    SOURCE_FILE="${PROOF_DIR}/reset.cpp"
    while IFS='|' read -r FUNCTION_NAME FLAGS EXPECTED; do
        read -r -a ARGUMENTS <<< "${FLAGS}"
        PROFILE="regression-reset-boundary-${RESULT_COUNT}"
        verify "${FUNCTION_NAME}" --proof-unwind 226 "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${RESULT_COUNT}.log" 2>&1
        RESULT_COUNT=$((RESULT_COUNT + 1))
        if [[ "${FAILED_CHECKS}" != "${RESULT_COUNT}" ]] ||
            ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/$((RESULT_COUNT - 1)).log" ||
            ! grep -F "${EXPECTED}" "${TEST_WORK_DIR}/$((RESULT_COUNT - 1)).log" | grep -q 'FAILED:'; then
            tail -n 60 "${TEST_WORK_DIR}/$((RESULT_COUNT - 1)).log" >&2
            fail "reset proof accepted an incompatible boundary: ${FUNCTION_NAME} ${FLAGS}"
        fi
    done <<'CASES'
verify_reset|-D FORMAL_ESKF_PROOF_RESET_CONTRACT=0|Runner error: reset caller requires its explicit callee summaries
verify_reset|-D FORMAL_ESKF_PROOF_RESET_CONTRACT=1|Runner error: reset caller requires its explicit callee summaries
verify_reset_sandwich|-D FORMAL_ESKF_PROOF_PRODUCT_CONTRACT=0|Runner error: actual sandwich requires only its product summary
verify_reset_sandwich|-D FORMAL_ESKF_PROOF_PRODUCT_CONTRACT=1 -D FORMAL_ESKF_PROOF_RESET_CONTRACT=1|Runner error: actual sandwich requires only its product summary
verify_covariance_product|-D FORMAL_ESKF_PROOF_PRODUCT_CONTRACT=1|Runner error: actual covariance product requires the matching entry boundary
verify_covariance_product|-D FORMAL_ESKF_PROOF_INS=1|Runner error: actual covariance product requires the matching entry boundary
verify_covariance_entry|-D FORMAL_ESKF_PROOF_INS=1|Runner error: actual INS entry requires dot and access summaries
verify_covariance_entry|-D FORMAL_ESKF_PROOF_INS=1 -D FORMAL_ESKF_PROOF_DOT_CONTRACT=1|Runner error: actual INS entry requires dot and access summaries
verify_covariance_entry|-D FORMAL_ESKF_PROOF_INS=1 -D FORMAL_ESKF_PROOF_ACCESS_CONTRACT=1|Runner error: actual INS entry requires dot and access summaries
verify_covariance_entry|-D FORMAL_ESKF_PROOF_INS=1 -D FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT=1 -D FORMAL_ESKF_PROOF_DOT_CONTRACT=1 -D FORMAL_ESKF_PROOF_ACCESS_CONTRACT=1|Runner error: actual INS entry requires dot and access summaries
verify_covariance_dot|-D FORMAL_ESKF_PROOF_DOT_CONTRACT=1|Runner error: covariance dot producer must be actual
verify_reset_finish|-D FORMAL_ESKF_PROOF_FINITE_CONTRACT=0|Runner error: actual covariance finalizer requires only the finite-predicate summary
verify_reset_finish|-D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1 -D FORMAL_ESKF_PROOF_RESET_CONTRACT=1|Runner error: actual covariance finalizer requires only the finite-predicate summary
verify_reset_finite|-D FORMAL_ESKF_PROOF_FINITE_CONTRACT=1|Runner error: covariance finite predicate must be actual
verify_reset_storage|-D FORMAL_ESKF_PROOF_INS=0|Runner error: reset storage producer requires actual INS storage
verify_reset_storage|-D FORMAL_ESKF_PROOF_INS=1 -D FORMAL_ESKF_PROOF_ACCESS_CONTRACT=1|Runner error: reset storage producer requires actual INS storage
CASES
    printf 'ESBMC reset guard tests: pass (%d incompatible boundaries rejected)\n' "${RESULT_COUNT}"
)

check_reset_regressions()
(
    local TEST_WORK_DIR BINARY64 CASE RESULT EXPECTED LABEL FUNCTION_NAME
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-reset-regression.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    for BINARY64 in 0 1; do
        for CASE in copy fault default comparison; do
            EXPECTED=0
            LABEL='VERIFICATION SUCCESSFUL'
            FUNCTION_NAME=verify_reset_regression
            ARGUMENTS=(-D FORMAL_ESKF_PROOF_INS=1 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
            case "${CASE}" in
                comparison) FUNCTION_NAME=verify_reset_comparison ;;
                fault)
                    EXPECTED=1
                    LABEL='E-RESET regression: copied IEEE values and the last covariance cell are checked'
                    ARGUMENTS+=(-D FORMAL_ESKF_TEST_FAULT=1)
                    ;;
                default)
                    EXPECTED=1
                    LABEL='E-RESET regression: the default is the closed-form right Jacobian'
                    ARGUMENTS+=(-D ESKF_RESET_APPROX=1)
                    ;;
            esac
            RESULT=0
            "${ESBMC_COMMAND}" "${TEST_DIR}/esbmc/reset_checks.cpp" "${ESBMC_ARGUMENTS[@]}" \
                --unwind 226 --timeout 120s --memlimit 4g --function "${FUNCTION_NAME}" \
                "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" 2>&1 || RESULT=$?
            if [[ "${RESULT}" != "${EXPECTED}" ]] || ! grep -Fq "${LABEL}" "${TEST_WORK_DIR}/${BINARY64}-${CASE}.log"; then
                tail -n 60 "${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" >&2
                fail "reset verifier regression failed: binary64=${BINARY64}, ${CASE}"
            fi
        done
    done
    printf 'ESBMC reset regressions: pass (IEEE copies; last-cell faults; default mode; all-cell comparison)\n'
)

check_pcov_contract_guards()
(
    local TEST_WORK_DIR FILE FUNCTION_NAME FLAGS EXPECTED RESULT_COUNT=0 RESULT
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-pcov-guard.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    while IFS='|' read -r FILE FUNCTION_NAME FLAGS EXPECTED; do
        read -r -a ARGUMENTS <<< "${FLAGS}"
        RESULT=0
        "${ESBMC_COMMAND}" "${PROOF_DIR}/${FILE}" "${ESBMC_ARGUMENTS[@]}" \
            --unwind 10 --timeout 120s --memlimit 4g --function "${FUNCTION_NAME}" \
            "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${RESULT_COUNT}.log" 2>&1 || RESULT=$?
        if [[ "${RESULT}" != 1 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${RESULT_COUNT}.log" ||
            ! grep -F 'FAILED:' "${TEST_WORK_DIR}/${RESULT_COUNT}.log" | grep -Fq "${EXPECTED}"; then
            tail -n 30 "${TEST_WORK_DIR}/${RESULT_COUNT}.log" >&2
            fail "covariance prediction guard did not reject the incompatible boundary: ${FUNCTION_NAME} ${FLAGS}"
        fi
        RESULT_COUNT=$((RESULT_COUNT + 1))
    done <<'CASES'
covariance_prediction.cpp|verify_predict_covariance|-D FORMAL_ESKF_PROOF_PCOV_CONTRACT=0|Runner error: E-PCOV requires only its explicit caller summaries
covariance_prediction.cpp|verify_predict_covariance|-D FORMAL_ESKF_PROOF_PCOV_CONTRACT=1 -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1|Runner error: E-PCOV requires only its explicit caller summaries
covariance_transition.cpp|verify_covariance_transition|-D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=0|Runner error: transition producer requires only Exp and rotation summaries
covariance_transition.cpp|verify_covariance_transition|-D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=1 -D FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT=1|Runner error: transition producer requires only Exp and rotation summaries
covariance_transition.cpp|verify_covariance_rotation|-D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=1|Runner error: covariance rotation producer must execute the actual implementation
covariance_transition.cpp|verify_covariance_quaternion_validation|-D FORMAL_ESKF_PROOF_TRANSITION_CONTRACT=1|Runner error: covariance quaternion validation must execute the actual implementation
CASES
    printf 'ESBMC covariance prediction guards: pass (%d incompatible boundaries rejected)\n' "${RESULT_COUNT}"
)

check_pcov_regressions()
(
    local TEST_WORK_DIR BINARY64 CASE RESULT EXPECTED LABEL
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-pcov-regression.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    for BINARY64 in 0 1; do
        for CASE in ieee fault default; do
            EXPECTED=0
            LABEL='VERIFICATION SUCCESSFUL'
            ARGUMENTS=(-D FORMAL_ESKF_PROOF_INS=1 -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}")
            case "${CASE}" in
                fault)
                    EXPECTED=1
                    LABEL='E-PCOV: all noise blocks and every unaffected covariance entry reach finalization'
                    ARGUMENTS+=(-D FORMAL_ESKF_TEST_FAULT=1)
                    ;;
                default)
                    EXPECTED=1
                    LABEL='E-PCOV regression: default prediction uses Exp'
                    ARGUMENTS+=(-D ESKF_QUAT_APPROX=1)
                    ;;
            esac
            RESULT=0
            "${ESBMC_COMMAND}" "${TEST_DIR}/esbmc/pcov_checks.cpp" "${ESBMC_ARGUMENTS[@]}" \
                --unwind 226 --timeout 120s --memlimit 4g --function verify_pcov_regression \
                "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" 2>&1 || RESULT=$?
            if [[ "${RESULT}" != "${EXPECTED}" ]] || ! grep -Fq "${LABEL}" "${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" ||
                { ((EXPECTED)) && ! grep -F 'FAILED:' "${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" | grep -Fq "${LABEL}"; }; then
                tail -n 30 "${TEST_WORK_DIR}/${BINARY64}-${CASE}.log" >&2
                fail "covariance prediction regression failed: binary64=${BINARY64}, ${CASE}"
            fi
        done
    done
    printf 'ESBMC covariance prediction regressions: pass (IEEE candidate, last-cell faults and default mode; both scalars)\n'
)

check_step_contract_guards()
(
    local TEST_WORK_DIR FUNCTION_NAME FLAGS EXPECTED RESULT COUNT=0
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-step-guard.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    while IFS='|' read -r FUNCTION_NAME FLAGS EXPECTED; do
        read -r -a ARGUMENTS <<< "${FLAGS}"
        RESULT=0
        "${ESBMC_COMMAND}" "${PROOF_DIR}/step.cpp" "${ESBMC_ARGUMENTS[@]}" \
            --unwind 10 --timeout 120s --memlimit 4g --function "${FUNCTION_NAME}" \
            "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${COUNT}.log" 2>&1 || RESULT=$?
        if [[ "${RESULT}" != 1 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${COUNT}.log" ||
            ! grep -F 'FAILED:' "${TEST_WORK_DIR}/${COUNT}.log" | grep -Fq "${EXPECTED}"; then
            tail -n 30 "${TEST_WORK_DIR}/${COUNT}.log" >&2
            fail "step guard did not reject the incompatible boundary: ${FUNCTION_NAME} ${FLAGS}"
        fi
        COUNT=$((COUNT + 1))
    done <<'CASES'
verify_prediction_step|-D FORMAL_ESKF_PROOF_STEP_CONTRACT=0|Runner error: step caller requires only component summaries
verify_injection_reset_step|-D FORMAL_ESKF_PROOF_STEP_CONTRACT=0|Runner error: step caller requires only component summaries
verify_prediction_step|-D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 -D FORMAL_ESKF_PROOF_STEP_FRAME=1|Runner error: step caller requires only component summaries
verify_nominal_prediction_frame|-D FORMAL_ESKF_PROOF_STEP_FRAME=0|Runner error: nominal frame producer requires only quaternion leaf summaries
verify_nominal_prediction_frame|-D FORMAL_ESKF_PROOF_STEP_FRAME=1 -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1|Runner error: nominal frame producer requires only quaternion leaf summaries
verify_prediction_step|-D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 -D FORMAL_ESKF_PROOF_STEP_ALIAS=4|Runner error: prediction has exactly four alias arrangements
verify_injection_reset_step|-D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 -D FORMAL_ESKF_PROOF_STEP_ALIAS=8|Runner error: injection/reset has exactly eight alias arrangements
CASES
    printf 'ESBMC step guards: pass (%d incompatible boundaries/aliases rejected)\n' "${COUNT}"
)

check_step_regressions()
(
    local TEST_WORK_DIR BINARY64
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-step-regression.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    for BINARY64 in 0 1; do
        if ! "${ESBMC_COMMAND}" "${TEST_DIR}/esbmc/step_checks.cpp" "${ESBMC_ARGUMENTS[@]}" \
            --unwind 226 --timeout 120s --memlimit 4g --function verify_step_regression \
            -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D FORMAL_ESKF_PROOF_INS=1 \
            -D FORMAL_ESKF_PROOF_STEP_CONTRACT=1 -D FORMAL_ESKF_PROOF_STEP_ALIAS=7 \
            --multi-property >"${TEST_WORK_DIR}/${BINARY64}.log" 2>&1 ||
            ! grep -q 'VERIFICATION SUCCESSFUL' "${TEST_WORK_DIR}/${BINARY64}.log"; then
            tail -n 30 "${TEST_WORK_DIR}/${BINARY64}.log" >&2
            fail "step signed-zero/non-finite/full-in-place fixture failed: binary64=${BINARY64}"
        fi
    done
    printf 'ESBMC step regressions: pass (both transactions, symbolic statuses and full in-place output; both scalars)\n'
)

check_solve_contract_guards()
(
    local TEST_WORK_DIR SOURCE_NAME FUNCTION_NAME FLAGS EXPECTED RESULT COUNT=0
    local -a ARGUMENTS
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-solve-guard.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    while IFS='|' read -r SOURCE_NAME FUNCTION_NAME FLAGS EXPECTED; do
        read -r -a ARGUMENTS <<< "${FLAGS}"
        RESULT=0
        "${ESBMC_COMMAND}" "${PROOF_DIR}/${SOURCE_NAME}" "${ESBMC_ARGUMENTS[@]}" \
            --unwind 10 --timeout 120s --memlimit 4g --function "${FUNCTION_NAME}" \
            -D FORMAL_ESKF_PROOF_SIZE=1 -D FORMAL_ESKF_PROOF_COLUMNS=1 \
            "${ARGUMENTS[@]}" --multi-property >"${TEST_WORK_DIR}/${COUNT}.log" 2>&1 || RESULT=$?
        if [[ "${RESULT}" != 1 ]] || ! grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${COUNT}.log" ||
            ! grep -F 'FAILED:' "${TEST_WORK_DIR}/${COUNT}.log" | grep -Fq "${EXPECTED}"; then
            tail -n 30 "${TEST_WORK_DIR}/${COUNT}.log" >&2
            fail "solve guard did not reject the incompatible boundary: ${FUNCTION_NAME} ${FLAGS}"
        fi
        COUNT=$((COUNT + 1))
    done <<'CASES'
solve.cpp|verify_cholesky_arithmetic|-D FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT=1|Runner error: arithmetic producer must be actual
solve_coefficients.cpp|verify_factor_equations||Runner error: equations require arithmetic contracts
solve_coefficients.cpp|verify_forward_equations||Runner error: equations require arithmetic contracts
solve_coefficients.cpp|verify_backward_equations||Runner error: equations require arithmetic contracts
solve_flow.cpp|verify_solve_orchestration||Runner error: orchestration requires only helper summaries
solve_flow.cpp|verify_left_solve||Runner error: left solve requires kernel and symmetry summaries
solve_flow.cpp|verify_right_solve||Runner error: right solve requires only left solve summary
solve_flow.cpp|verify_solve_symmetry|-D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=2|Runner error: symmetry producer must be actual
solve_flow.cpp|verify_solve_orchestration|-D FORMAL_ESKF_PROOF_SOLVE_BOUNDARY=1 -D FORMAL_ESKF_PROOF_ALIAS=2|Runner error: kernel supports separate/identical read-only inputs
CASES
    printf 'ESBMC solve guards: pass (%d incompatible boundaries/aliases rejected)\n' "${COUNT}"
)

check_solve_regressions()
(
    local TEST_WORK_DIR BINARY64 FAULT RESULT
    TEST_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/formal-eskf-solve-regression.XXXXXX")"
    trap 'rm -r -- "${TEST_WORK_DIR}"' EXIT
    select_esbmc
    check_esbmc_version
    for BINARY64 in 0 1; do
        for FAULT in 0 1; do
            RESULT=0
            "${ESBMC_COMMAND}" "${TEST_DIR}/esbmc/solve_checks.cpp" "${ESBMC_ARGUMENTS[@]}" \
                --unwind 10 --timeout 120s --memlimit 4g --function verify_solve_regression \
                -D FORMAL_ESKF_PROOF_SIZE=2 -D FORMAL_ESKF_PROOF_COLUMNS=3 \
                -D "FORMAL_ESKF_PROOF_BINARY64=${BINARY64}" -D "FORMAL_ESKF_PROOF_CHECK_FAULT=${FAULT}" \
                --multi-property >"${TEST_WORK_DIR}/${BINARY64}-${FAULT}.log" 2>&1 || RESULT=$?
            if ((FAULT == 0)); then
                [[ "${RESULT}" == 0 ]] && grep -q 'VERIFICATION SUCCESSFUL' "${TEST_WORK_DIR}/${BINARY64}-${FAULT}.log" ||
                    fail 'Cholesky success/failure regression did not pass'
            else
                [[ "${RESULT}" != 0 ]] && grep -q 'VERIFICATION FAILED' "${TEST_WORK_DIR}/${BINARY64}-${FAULT}.log" &&
                    grep -q 'Regression: all solution coefficients, including last-cell faults' "${TEST_WORK_DIR}/${BINARY64}-${FAULT}.log" ||
                    fail 'Cholesky last-cell fault was not detected'
            fi
        done
    done
    printf 'ESBMC solve regressions: pass (success/failure witnesses and last-cell faults; both scalars)\n'
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
check_reset_arguments
check_pcov_arguments
check_step_arguments
check_solve_arguments
check_solver_argument_guard
printf 'ESBMC inventory tests: pass (790 profiles; 93 entry points; all shards)\n'
if ((RUN_SOLVER)); then
    check_mode_constants
    check_rotation_contract_guards
    check_normalization_contract_guards
    check_maps_contract_guards
    check_noise_contract_guard
    check_injection_contract_guards
    check_jacobian_solver
    check_jacobian_contract_guards
    check_reset_contract_guards
    check_reset_regressions
    check_pcov_contract_guards
    check_pcov_regressions
    check_step_contract_guards
    check_step_regressions
    check_solve_contract_guards
    check_solve_regressions
fi
