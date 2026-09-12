## Summary

Define the source-level defect and portability checks for the shared C++ core. Cppcheck complements compiler warnings, tests and formal verification; it does not replace them.

This follows the [C++ portability constraints](cpp-portability-and-constraints.md). The executable configuration is maintained in [configs/cppcheck.cppcheck](../configs/cppcheck.cppcheck) and [verify_cppcheck.sh](../scripts/verify_cppcheck.sh).

## Analysis boundary

Analyze the header-based core through CMake's compilation database and the translation units that instantiate its templates. Evidence applies only to the recorded sources, definitions, scalar types and backends; scanning headers does not establish coverage of every possible instantiation.

Check both `unix32` and `unix64` data models with separate analysis caches. These are portability checks, not substitutes for a deployment's compiler, ABI or target-specific analysis.

## Required checks

| Check | Purpose |
|---|---|
| `error` and `warning` | Detect invalid accesses, uninitialized values, undefined behavior and suspicious execution paths. |
| `style` and `performance` | Identify error-prone constructs and avoidable work; review findings in the context of fixed-size embedded code. |
| `portability` | Detect dependencies on platform widths and implementation-defined behavior. |
| Exhaustive value flow, CTU analysis and public/external safe checks | Analyze bounded call chains under conservative assumptions about library callers. |
| `threadsafety` and `misc` addons | Detect shared-state hazards, known unsafe APIs and additional suspicious C++ constructs. |
| Configuration validation, `--safety` and checker-coverage validation | Detect incomplete analysis and unexpected loss of enabled checks. |

Pin the tool version and explicitly select checks. The script owns exact flags, analysis depth and the expected checker inventory; these must be reviewed together when upgrading the tool.

Every enabled diagnostic is blocking. Resolve it or document a narrowly scoped suppression; broad category or file suppressions are not acceptable. Analysis failures must not be reported as clean source results.

## Checks outside the default gate

| Check | Reason |
|---|---|
| `information` | Keep analysis metadata in checker reports rather than treating it as a source defect. |
| `unusedFunction` and unused templates | In-tree use is not a reliable measure of whether a public library API is needed downstream. |
| `--inconclusive` | Keep lower-confidence findings in an explicit audit until individual diagnostics are accepted into the gate. |
| Naming, cast-reporting and platform-specific addons | Require an applicable project policy or API scope before enabling them. |

Do not replace the explicit selection with `--enable=all`. Compiler warnings remain necessary because their coverage differs from Cppcheck's.

## MISRA policy

The MISRA C addon is not evidence of MISRA C++ compliance. A future MISRA C++ gate requires an agreed governing standard, a compatible language profile, a checker with documented coverage, and explicit handling of manual rules and justified deviations.

Keep that gate separate from open-source Cppcheck. Do not mix MISRA, CERT and AUTOSAR profiles without a defined scope, silently exclude inconvenient rules, or copy licensed rule text into the repository.

## Claim boundary

A passing run means that the configured analysis completed without enabled diagnostics for the selected translation units and data models. It does not prove absence of defects, all template/backend configurations, numerical accuracy, concurrency safety, MISRA conformance or target-machine behavior.

Run the configured gate with:

```sh
./scripts/verify_cppcheck.sh
```
