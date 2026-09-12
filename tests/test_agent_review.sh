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
         tool_results: ["lean", "esbmc"] |
            map({tool: ., status: "pass", exit_code: 0, summary: "Not executed."}),
         requirements: [.requirements[] |
            {id: .id, classification: "PASS", summary: "Synthetic claim.",
             evidence: [.references | to_entries[] |
                {layer: .key, file: .value[0].file, line: 1,
                 symbol: .value[0].symbol, observation: "Synthetic citation."}],
             assumption_differences: [], findings: [], next_action: ""}],
         unmapped_references: [], limitations: ["No verifiers or agent were run."]}
    ' "${TRACEABILITY_MAP}" >"${TEST_ROOT}/base-report.json"

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
    run_verifiers() { :; }
    validate_execution_inventory() { :; }
    run_codex_review() { :; }
    main
}

run_mocked_failed_verifier()
{
    LEAN_RESULT=fail
    LEAN_STATUS=9
    run_mocked_review
}

test_exit_status()
{
    cp "${TEST_ROOT}/base-report.json" "${REPORT_FILE}"
    expect_status 'PASS exits zero' 0 run_mocked_review
    jq '.requirements[0].classification = "GAP" | .overall = "incomplete"' \
        "${TEST_ROOT}/base-report.json" >"${REPORT_FILE}"
    expect_status 'GAP exits two' 2 run_mocked_review
    jq '.requirements[0].classification = "MISMATCH" | .overall = "fail"' \
        "${TEST_ROOT}/base-report.json" >"${REPORT_FILE}"
    expect_status 'MISMATCH exits one' 1 run_mocked_review
    jq '.tool_results[0].status = "fail" | .tool_results[0].exit_code = 9 | .overall = "fail"' \
        "${TEST_ROOT}/base-report.json" >"${REPORT_FILE}"
    expect_status 'verifier failure exits one' 1 run_mocked_failed_verifier
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
expect_status 'repository map references' 0 check_repository_map
printf '%bAgent-review regression tests: %d passed%b\n' "${PASS_COLOR}" "${TEST_COUNT}" "${NO_COLOR}"
