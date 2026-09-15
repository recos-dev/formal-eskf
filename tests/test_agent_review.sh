#!/usr/bin/env bash

set -euo pipefail

TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/run_agent_review.sh
source "${TEST_DIR}/../scripts/run_agent_review.sh"

SOURCE_REPO_DIR="${REPO_DIR}"
TEST_ROOT="$(mktemp -d /tmp/formal-eskf-agent-test.XXXXXX)"
TEST_COUNT=0

clean_up_tests()
{
    rm -rf -- "${TEST_ROOT}"
}

trap clean_up_tests EXIT

expect_status()
{
    local LABEL="$1" EXPECTED="$2" ACTUAL=0
    shift 2

    ("$@") >"${TEST_ROOT}/case.log" 2>&1 || ACTUAL=$?
    if [[ "${EXPECTED}" != "${ACTUAL}" ]]; then
        printf 'FAIL: %s (expected exit %s, got %s)\n' "${LABEL}" "${EXPECTED}" "${ACTUAL}" >&2
        tail -n 30 "${TEST_ROOT}/case.log" >&2
        exit 1
    fi
    TEST_COUNT=$((TEST_COUNT + 1))
}

prepare_fixtures()
{
    # Synthetic validator inputs only: these are never proof evidence.
    REPO_DIR="${TEST_ROOT}/repo"
    REVIEW_WORK_DIR="${TEST_ROOT}/work"
    TRACEABILITY_MAP="${TEST_ROOT}/map.json"
    REPORT_FILE="${TEST_ROOT}/report.json"
    AUDIT_FILE="${REVIEW_WORK_DIR}/audit.json"
    AUDIT_SCHEMA="${REVIEW_WORK_DIR}/audit-schema.json"
    mkdir -p "${REPO_DIR}" "${REVIEW_WORK_DIR}" "${TEST_ROOT}/bin"
    printf 'synthetic witness\n' >"${REPO_DIR}/evidence.txt"
    cp "${REPO_DIR}/evidence.txt" "${REPO_DIR}/unmapped.txt"

    jq -n '
        def references:
            ["spec", "lean", "cpp", "esbmc"] |
            map({key: ., value: [{file: "evidence.txt", symbol: "synthetic witness"}]}) |
            from_entries;
        {version: 1, requirements: ["R-ONE", "R-TWO"] |
            map({id: ., title: "Synthetic requirement", references: references})}
    ' >"${TEST_ROOT}/base-map.json"
    cp "${TEST_ROOT}/base-map.json" "${TRACEABILITY_MAP}"

    jq '
        {overall: "pass", summary: "Synthetic validator fixture, not an audit.",
         tool_results: ["lean", "esbmc", "cppcheck", "asan_ubsan"] |
            map({tool: ., status: "pass", exit_code: 0, summary: "Not executed."}),
         requirements: [.requirements[] |
            {id: .id, classification: "PASS", summary: "Synthetic claim.",
             evidence: [.references | to_entries[] |
                {layer: .key, file: .value[0].file, line: 1,
                 symbol: .value[0].symbol, observation: "Synthetic citation."}],
             assumption_differences: [], findings: [], next_action: ""}],
         unmapped_references: [], limitations: ["No verifiers or agent were run."]}
    ' "${TRACEABILITY_MAP}" >"${TEST_ROOT}/base-report.json"
    jq 'del(.overall, .tool_results)' "${TEST_ROOT}/base-report.json" >"${TEST_ROOT}/base-audit.json"
    write_audit_schema

    # Expand these variables in the generated stub, not in this test process.
    # shellcheck disable=SC2016
    printf '%s\n' '#!/usr/bin/env bash' \
        '[[ "${1:-}" == --list ]] || exit 64' \
        'exec awk 1 "${0%/*}/plan.tsv"' >"${TEST_ROOT}/bin/verify_esbmc.sh"
    chmod +x "${TEST_ROOT}/bin/verify_esbmc.sh"
    printf 'PLAN\tformal/fixture.cpp\tbinary32\tverify_fixture\nPLAN\tformal/fixture.cpp\tbinary64\tverify_fixture\n' \
        >"${TEST_ROOT}/base-plan.tsv"
    awk -F '\t' -v OFS='\t' '{$1="RESULT"; print $0, 0}' \
        "${TEST_ROOT}/base-plan.tsv" >"${TEST_ROOT}/base-results.tsv"
}

check_report()
{
    local LABEL="$1" EXPECTED="$2" FILTER="$3"
    jq "${FILTER}" "${TEST_ROOT}/base-report.json" >"${REPORT_FILE}"
    expect_status "${LABEL}" "${EXPECTED}" validate_review_report
}

test_reports()
{
    check_report 'complete report' 0 '.'
    check_report 'empty evidence cannot pass' 1 '.requirements[].evidence = []'
    check_report 'missing evidence layer cannot pass' 1 '.requirements[0].evidence |= map(select(.layer != "lean"))'
    check_report 'unmapped file cannot stand in for a layer' 1 '.requirements[0].evidence[0].file = "unmapped.txt"'
    check_report 'invalid classification' 1 '.requirements[0].classification = "INVALID"'
    check_report 'missing required field' 1 'del(.requirements[0].findings)'
    check_report 'unknown field' 1 '.unexpected = true'
    check_report 'invalid evidence layer' 1 '.requirements[0].evidence[0].layer = "other"'
    check_report 'empty symbol' 1 '.requirements[0].evidence[0].symbol = ""'
    check_report 'whitespace symbol' 1 '.requirements[0].evidence[0].symbol = " "'
    check_report 'invalid line number' 1 '.requirements[0].evidence[0].line = 0'
    check_report 'fractional line number' 1 '.requirements[0].evidence[0].line = 1.5'
    check_report 'wrong source line' 1 '.requirements[0].evidence[0].line = 2'
    check_report 'wrong symbol' 1 '.requirements[0].evidence[0].symbol = "absent"'
    check_report 'unsafe path' 1 '.requirements[0].evidence[0].file = "../outside.txt" | .requirements[0].classification = "GAP" | .overall = "incomplete"'
    check_report 'missing requirement' 1 '.requirements |= .[0:1]'
    check_report 'duplicate requirement' 1 '.requirements[1] = .requirements[0]'
    check_report 'reordered requirements' 1 '.requirements |= reverse'
    check_report 'changed verifier result' 1 '.tool_results[0].exit_code = 1'
    check_report 'duplicate tool result' 1 '.tool_results[1] = .tool_results[0]'
    check_report 'old two-tool report rejected' 1 '.tool_results |= .[0:2]'
    check_report 'reordered tools rejected' 1 '.tool_results |= reverse'
    check_report 'changed Cppcheck status rejected' 1 '.tool_results[2].status = "fail" | .overall = "fail"'
    check_report 'changed Cppcheck exit code rejected' 1 '.tool_results[2].exit_code = 2'
    check_report 'Cppcheck is not a proof layer' 1 '.requirements[0].evidence[0].layer = "cppcheck"'
    check_report 'missing sanitizer result rejected' 1 '.tool_results |= .[0:3]'
    check_report 'changed sanitizer status rejected' 1 '.tool_results[3].status = "fail" | .overall = "fail"'
    check_report 'changed sanitizer exit code rejected' 1 '.tool_results[3].exit_code = 8'
    check_report 'sanitizers are not a proof layer' 1 '.requirements[0].evidence[0].layer = "asan_ubsan"'
    check_report 'error finding cannot pass' 1 '.requirements[0].findings = [{category:"coverage", severity:"error", description:"Missing evidence."}]'
    check_report 'inconsistent overall status' 1 '.overall = "fail"'
    check_report 'valid numerical gap' 0 '.requirements[0].classification = "NUMERICAL" | .overall = "incomplete"'
    check_report 'valid mismatch' 0 '.requirements[0].classification = "MISMATCH" | .overall = "fail"'
    printf '{' >"${REPORT_FILE}"
    expect_status 'malformed JSON' 1 validate_review_report

    jq '.requirements[1].references.spec = []' "${TEST_ROOT}/base-map.json" >"${TRACEABILITY_MAP}"
    expect_status 'empty map layer remains reviewable' 0 validate_traceability_map
    check_report 'missing spec cannot pass even with a citation' 1 '.'
    check_report 'missing spec is not a numerical gap' 1 '.requirements[1].classification = "NUMERICAL" | .overall = "incomplete"'
    check_report 'missing spec stays GAP' 0 '.requirements[1].classification = "GAP" | .requirements[1].evidence = [] | .overall = "incomplete"'
    check_report 'missing spec can coexist with mismatch' 0 '.requirements[1].classification = "MISMATCH" | .overall = "fail"'
    cp "${TEST_ROOT}/base-map.json" "${TRACEABILITY_MAP}"
}

test_map()
{
    local FILTER
    expect_status 'valid map' 0 validate_traceability_map
    for FILTER in \
        '.requirements[0].references.spec[0].symbol = ""' \
        '.requirements[0].references.spec[0].symbol = "absent"' \
        '.requirements[0].references.spec[0].file = "missing.txt"' \
        '.requirements[0].references.spec[0].file = "../outside.txt"' \
        'del(.requirements[0].references.lean)' \
        '.requirements[1].id = .requirements[0].id'; do
        jq "${FILTER}" "${TEST_ROOT}/base-map.json" >"${TRACEABILITY_MAP}"
        expect_status "invalid map: ${FILTER}" 1 validate_traceability_map
    done
    cp "${TEST_ROOT}/base-map.json" "${TRACEABILITY_MAP}"
}

check_inventory()
{
    local LABEL="$1" EXPECTED="$2" PLAN_FILTER="$3" RESULT_FILTER="$4"
    awk -F '\t' -v OFS='\t' "${PLAN_FILTER}" "${TEST_ROOT}/base-plan.tsv" >"${TEST_ROOT}/bin/plan.tsv"
    awk -F '\t' -v OFS='\t' "${RESULT_FILTER}" "${TEST_ROOT}/base-results.tsv" >"${TEST_ROOT}/results.tsv"
    expect_status "${LABEL}" "${EXPECTED}" inspect_inventory
}

inspect_inventory()
{
    SCRIPT_DIR="${TEST_ROOT}/bin"
    ESBMC_LOG="${TEST_ROOT}/results.tsv"
    ESBMC_RESULT=pass
    ESBMC_STATUS=0
    validate_execution_inventory
    [[ "${ESBMC_RESULT}" == pass && "${ESBMC_STATUS}" == 0 ]]
}

test_inventory()
{
    check_inventory 'complete inventory' 0 '1' '1'
    check_inventory 'missing execution' 1 '1' 'NR == 1'
    check_inventory 'duplicate execution' 1 '1' '{print; print}'
    # The dollar references are awk fields, not shell variables.
    # shellcheck disable=SC2016
    check_inventory 'failed execution' 1 '1' '{$5=7; print}'
    # shellcheck disable=SC2016
    check_inventory 'noncanonical exit status' 1 '1' '{$5="0.0"; print}'
    check_inventory 'malformed result' 1 '1' 'NF=4'
    check_inventory 'empty actual inventory' 1 '1' '0'
    check_inventory 'empty expected inventory' 1 '0' '0'
    check_inventory 'malformed plan' 1 'NF=3' '1'
    check_inventory 'duplicate expected and actual profiles' 1 '{print; print}' '{print; print}'
}

run_mocked_review()
{
    # Only external execution is replaced; the real validation chain and exit
    # status logic run against the synthetic files prepared above.
    check_prerequisites() { :; }
    prepare_output() { :; }
    # Called indirectly by the sourced runner's EXIT handler.
    # shellcheck disable=SC2317
    clean_up() { :; }
    run_verifiers() { :; }
    validate_execution_inventory() { :; }
    run_codex_review() { :; }
    main
}

run_mocked_failed_check()
{
    case "$1" in
        lean) LEAN_RESULT=fail; LEAN_STATUS=9 ;;
        esbmc) ESBMC_RESULT=fail; ESBMC_STATUS=7 ;;
        cppcheck) CPPCHECK_RESULT=fail; CPPCHECK_STATUS=2 ;;
        asan_ubsan) SANITIZER_RESULT=fail; SANITIZER_STATUS=8 ;;
        *) return 99 ;;
    esac
    run_mocked_review
}

check_final_elapsed_time()
{
    tail -n 1 "${TEST_ROOT}/case.log" | rg -q '^Total elapsed time: [0-9]{2,}:[0-9]{2}:[0-9]{2}$'
    [[ "$(rg -c '^Total elapsed time:' "${TEST_ROOT}/case.log")" == 1 ]]
    TEST_COUNT=$((TEST_COUNT + 1))
}

test_exit_status()
{
    local TOOL EXPECTED_STATUS
    cp "${TEST_ROOT}/base-audit.json" "${AUDIT_FILE}"
    expect_status 'PASS exits zero' 0 run_mocked_review
    check_final_elapsed_time
    jq '.requirements[0].classification = "GAP"' "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
    expect_status 'GAP exits two' 2 run_mocked_review
    check_final_elapsed_time
    jq '.requirements[0].classification = "NUMERICAL"' "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
    expect_status 'NUMERICAL exits two' 2 run_mocked_review
    check_final_elapsed_time
    jq '.requirements[0].classification = "MISMATCH"' "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
    expect_status 'MISMATCH exits one' 1 run_mocked_review
    check_final_elapsed_time
    cp "${TEST_ROOT}/base-audit.json" "${AUDIT_FILE}"
    for TOOL in lean esbmc cppcheck asan_ubsan; do
        case "${TOOL}" in
            lean) EXPECTED_STATUS=9 ;;
            esbmc) EXPECTED_STATUS=7 ;;
            cppcheck) EXPECTED_STATUS=2 ;;
            asan_ubsan) EXPECTED_STATUS=8 ;;
        esac
        expect_status "${TOOL} failure exits one even when every requirement passes" 1 run_mocked_failed_check "${TOOL}"
        check_final_elapsed_time
        expect_status "runner records ${TOOL} failure without changing the audit" 0 \
            check_assembled_failure "${TOOL}" "${EXPECTED_STATUS}"
    done
    jq '.requirements[0].classification = "GAP"' "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
    expect_status 'quality-check failure takes precedence over GAP' 1 run_mocked_failed_check cppcheck
    check_final_elapsed_time
}

check_assembled_failure()
{
    jq -e --arg TOOL "$1" --argjson EXPECTED_STATUS "$2" '
        .overall == "fail" and
        any(.tool_results[]; .tool == $TOOL and .status == "fail" and .exit_code == $EXPECTED_STATUS) and
        all(.tool_results[] | select(.tool != $TOOL); .status == "pass" and .exit_code == 0)
    ' "${REPORT_FILE}" >/dev/null || return 1
    diff -u <(jq -S . "${AUDIT_FILE}") <(jq -S 'del(.overall, .tool_results)' "${REPORT_FILE}")
}

run_mocked_prerequisite_failure()
{
    # Called indirectly by the sourced runner's EXIT handler.
    # shellcheck disable=SC2317
    clean_up() { :; }
    check_prerequisites() { fail 'synthetic missing prerequisite'; }
    main
}

check_elapsed_format()
{
    [[ "$(print_elapsed_time "$1" 2>&1)" == "Total elapsed time: $2" ]]
}

test_elapsed_time()
{
    expect_status 'zero duration' 0 check_elapsed_format 0 '00:00:00'
    expect_status 'hour and minute rollover' 0 check_elapsed_format 3661 '01:01:01'
    expect_status 'duration over 24 hours' 0 check_elapsed_format 90061 '25:01:01'
    expect_status 'prerequisite failure exits one' 1 run_mocked_prerequisite_failure
    check_final_elapsed_time
}

check_verifier_sequence()
{
    local EXPECTED_LEAN="$1" EXPECTED_ESBMC="$2" EXPECTED_CPPCHECK="$3" EXPECTED_SANITIZER="$4"
    # Check orchestration without executing real tools or modifying sources.
    run_verifier()
    {
        local STEP="$1" NAME="$2" LOG_FILE="$3"
        shift 3
        printf '%s\t%s\t%s\t%s\n' "${STEP}" "${NAME}" "${LOG_FILE##*/}" "$*" \
            >>"${TEST_ROOT}/sequence.tsv"
        case "${NAME}" in
            Lean) return "${EXPECTED_LEAN}" ;;
            ESBMC) return "${EXPECTED_ESBMC}" ;;
            Cppcheck) return "${EXPECTED_CPPCHECK}" ;;
            ASan/UBSan) return "${EXPECTED_SANITIZER}" ;;
            *) return 99 ;;
        esac
    }
    printf '' >"${TEST_ROOT}/sequence.tsv"
    run_verifiers
    [[ "${LEAN_STATUS}" == "${EXPECTED_LEAN}" && "${ESBMC_STATUS}" == "${EXPECTED_ESBMC}" &&
        "${CPPCHECK_STATUS}" == "${EXPECTED_CPPCHECK}" && "${SANITIZER_STATUS}" == "${EXPECTED_SANITIZER}" ]] || return 1
    [[ "${LEAN_RESULT}" == "$([[ ${EXPECTED_LEAN} == 0 ]] && echo pass || echo fail)" ]] || return 1
    [[ "${ESBMC_RESULT}" == "$([[ ${EXPECTED_ESBMC} == 0 ]] && echo pass || echo fail)" ]] || return 1
    [[ "${CPPCHECK_RESULT}" == "$([[ ${EXPECTED_CPPCHECK} == 0 ]] && echo pass || echo fail)" ]] || return 1
    [[ "${SANITIZER_RESULT}" == "$([[ ${EXPECTED_SANITIZER} == 0 ]] && echo pass || echo fail)" ]] || return 1
    printf '1\tLean\tlean.log\t%s/verify_lean.sh --wfail\n2\tESBMC\tesbmc.log\t%s/verify_esbmc.sh\n3\tCppcheck\tcppcheck.log\t%s/verify_cppcheck.sh\n4\tASan/UBSan\tsanitizers.log\t%s/verify_asan.sh\n' \
        "${SCRIPT_DIR}" "${SCRIPT_DIR}" "${SCRIPT_DIR}" "${SCRIPT_DIR}" >"${TEST_ROOT}/expected-sequence.tsv"
    diff -u "${TEST_ROOT}/expected-sequence.tsv" "${TEST_ROOT}/sequence.tsv"
}

check_verifier_output()
{
    local EXPECTED="$1" NAME="$2" STEP="$3" ACTUAL=0 OUTPUT REVIEW_WORK_DIR START_SECONDS
    REVIEW_WORK_DIR="$(mktemp -d "${TEST_ROOT}/verifier.XXXXXX")" || return 1
    START_SECONDS=${SECONDS}
    # Passed as a command argument to run_verifier.
    # shellcheck disable=SC2317
    synthetic_check_output()
    {
        local INDEX
        printf 'synthetic first line\n'
        for ((INDEX = 1; INDEX <= 80; INDEX++)); do
            printf 'synthetic detail %d\n' "${INDEX}"
        done
        printf 'synthetic final diagnostic\n' >&2
        return "${EXPECTED}"
    }
    OUTPUT="$(
        exec 2>&1
        trap finish_review EXIT
        run_verifier "${STEP}" "${NAME}" "${REVIEW_WORK_DIR}/check.log" synthetic_check_output
    )" || ACTUAL=$?
    [[ "${ACTUAL}" == "${EXPECTED}" && "${OUTPUT}" == *"[${STEP}/5] ${NAME}"* ]] || return 1
    [[ ! -e "${REVIEW_WORK_DIR}" ]] || return 1
    printf '%s\n' "${OUTPUT}" | tail -n 1 | rg -q '^Total elapsed time: [0-9]{2,}:[0-9]{2}:[0-9]{2}$' || return 1
    [[ "$(printf '%s\n' "${OUTPUT}" | rg -c '^Total elapsed time:')" == 1 ]] || return 1
    if ((EXPECTED == 0)); then
        [[ "${OUTPUT}" == *$'\e[32;01m'"${NAME}: pass"$'\e[0m'* &&
            "${OUTPUT}" != *'synthetic'* && "${OUTPUT}" != *'log tail:'* ]]
    else
        [[ "${OUTPUT}" == *$'\e[31;01m'"${NAME}: fail (exit ${EXPECTED})"$'\e[0m'* &&
            "${OUTPUT}" == *"${NAME} log tail:"* &&
            "${OUTPUT}" == *'synthetic final diagnostic'* &&
            "${OUTPUT}" != *'synthetic first line'* ]] || return 1
        [[ "$(printf '%s\n' "${OUTPUT}" | rg -c '^synthetic')" == 80 ]]
    fi
}

test_verifiers()
{
    expect_status 'four checks run in order without filtering sanitizer tests' 0 check_verifier_sequence 0 0 0 0
    expect_status 'earlier failures do not skip Cppcheck or sanitizers' 0 check_verifier_sequence 9 7 2 8
    expect_status 'Cppcheck success stays concise after cleanup and timing' 0 check_verifier_output 0 Cppcheck 3
    expect_status 'Cppcheck failure diagnostics survive cleanup with original status and timing' 0 check_verifier_output 2 Cppcheck 3
    expect_status 'sanitizer success stays concise after cleanup and timing' 0 check_verifier_output 0 ASan/UBSan 4
    expect_status 'sanitizer failure diagnostics survive cleanup with original status and timing' 0 check_verifier_output 8 ASan/UBSan 4
}

prepare_proof_logs()
{
    LEAN_LOG="${TEST_ROOT}/lean.log"
    ESBMC_LOG="${TEST_ROOT}/esbmc.log"
    printf 'synthetic Lean log\n' >"${LEAN_LOG}"
    printf 'synthetic ESBMC log\n' >"${ESBMC_LOG}"
}

check_review_prompt()
{
    prepare_proof_logs
    CPPCHECK_LOG="${TEST_ROOT}/cppcheck.log"
    SANITIZER_LOG="${TEST_ROOT}/sanitizers.log"
    printf 'synthetic Cppcheck pass\n' >"${CPPCHECK_LOG}"
    printf 'synthetic sanitizer pass\n' >"${SANITIZER_LOG}"
    write_review_prompt "${TEST_ROOT}/prompt-pass.txt"
    CPPCHECK_STATUS=2
    CPPCHECK_RESULT=fail
    SANITIZER_STATUS=8
    SANITIZER_RESULT=fail
    printf 'synthetic Cppcheck diagnostic\n' >"${CPPCHECK_LOG}"
    printf 'synthetic sanitizer diagnostic\n' >"${SANITIZER_LOG}"
    write_review_prompt "${TEST_ROOT}/prompt.txt"
    diff -u "${TEST_ROOT}/prompt-pass.txt" "${TEST_ROOT}/prompt.txt" &&
    ! rg -iq 'cppcheck|asan|ubsan|sanitizer' "${TEST_ROOT}/prompt.txt" &&
    rg -Fq 'Lean result: pass; exit code: 0' "${TEST_ROOT}/prompt.txt" &&
    rg -Fq 'ESBMC result: pass; exit code: 0' "${TEST_ROOT}/prompt.txt" &&
    rg -Fq "Full Lean log: ${LEAN_LOG}" "${TEST_ROOT}/prompt.txt" &&
    rg -Fq "Full ESBMC log: ${ESBMC_LOG}" "${TEST_ROOT}/prompt.txt" &&
    rg -Fq 'synthetic Lean log' "${TEST_ROOT}/prompt.txt" &&
    rg -Fq 'synthetic ESBMC log' "${TEST_ROOT}/prompt.txt" &&
    rg -Fq "Expected profile inventory: ${REVIEW_WORK_DIR}/expected-executions.tsv" "${TEST_ROOT}/prompt.txt" &&
    rg -Fq "Actual profile inventory: ${REVIEW_WORK_DIR}/actual-executions.tsv" "${TEST_ROOT}/prompt.txt" &&
    rg -Fq 'Return only the semantic audit:' "${TEST_ROOT}/prompt.txt"
}

check_audit_schema()
{
    jq -e '
        (.properties | keys) == ["limitations", "requirements", "summary", "unmapped_references"] and
        (.required | sort) == ["limitations", "requirements", "summary", "unmapped_references"] and
        .additionalProperties == false
    ' "${AUDIT_SCHEMA}" >/dev/null || return 1
    diff -u <(jq -S 'del(.properties.overall, .properties.tool_results) | .required -= ["overall", "tool_results"]' "${REPORT_SCHEMA}") \
        <(jq -S . "${AUDIT_SCHEMA}")
}

test_audit_assembly()
{
    local FILTER
    expect_status 'agent schema retains only semantic fields and their constraints' 0 check_audit_schema
    for FILTER in \
        '.overall = "pass"' \
        '.tool_results = []' \
        'del(.requirements)' \
        '.requirements[0].classification = "INVALID"'; do
        jq "${FILTER}" "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
        expect_status "reject invalid agent output before merging: ${FILTER}" 1 assemble_review_report
    done
    printf '{' >"${AUDIT_FILE}"
    expect_status 'reject malformed agent JSON before merging' 1 assemble_review_report
    cp "${TEST_ROOT}/base-report.json" "${AUDIT_FILE}"
    expect_status 'reject legacy agent-owned tool results and overall' 1 assemble_review_report
    jq '.requirements[0].classification = "GAP" | .requirements[0].evidence = [] |
        .requirements[0].findings = [{category: "coverage", severity: "error", description: "Synthetic gap."}] |
        .requirements[0].next_action = "Supply evidence."' "${TEST_ROOT}/base-audit.json" >"${AUDIT_FILE}"
    expect_status 'merge a semantic gap without weakening its findings' 0 check_assembled_audit
}

check_assembled_audit()
{
    assemble_review_report
    validate_review_report
    diff -u <(jq -S . "${AUDIT_FILE}") <(jq -S 'del(.overall, .tool_results)' "${REPORT_FILE}")
}

check_agent_invocation()
{
    # A local CLI stand-in checks the schema/output handoff, not AI semantics.
    # Invoked through CODEX_COMMAND by the sourced runner.
    # shellcheck disable=SC2317
    fixture_codex()
    {
        local SCHEMA_PATH="" OUTPUT_PATH=""
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --output-schema) SCHEMA_PATH="$2"; shift 2 ;;
                --output-last-message) OUTPUT_PATH="$2"; shift 2 ;;
                *) shift ;;
            esac
        done
        [[ "${SCHEMA_PATH}" == "${AUDIT_SCHEMA}" && "${OUTPUT_PATH}" == "${AUDIT_FILE}" &&
            "${OUTPUT_PATH}" != "${REPORT_FILE}" ]] || return 1
        check_audit_schema || return 1
        sed -n 'p' >"${TEST_ROOT}/agent-input.txt"
        cp "${TEST_ROOT}/base-audit.json" "${OUTPUT_PATH}"
    }
    CODEX_COMMAND=fixture_codex
    prepare_proof_logs
    run_codex_review
    diff -u "${REVIEW_WORK_DIR}/prompt.txt" "${TEST_ROOT}/agent-input.txt" || return 1
    check_assembled_audit
}

check_repository_map()
{
    REPO_DIR="${SOURCE_REPO_DIR}"
    TRACEABILITY_MAP="${SOURCE_REPO_DIR}/formal/agent-review/traceability-map.json"
    validate_traceability_map
}

prepare_fixtures
test_reports
test_map
test_inventory
test_exit_status
test_elapsed_time
test_verifiers
test_audit_assembly
expect_status 'quality-check results and logs do not enter or influence the agent prompt' 0 check_review_prompt
expect_status 'agent writes semantic output and runner assembles the final report' 0 check_agent_invocation
expect_status 'repository map references' 0 check_repository_map
printf '%bAgent-review regression tests: %d passed%b\n' "${PASS_COLOR}" "${TEST_COUNT}" "${NO_COLOR}"
