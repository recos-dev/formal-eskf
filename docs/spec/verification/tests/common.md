# Shared Core and Runtime Contracts

**Status:** Draft
**Scope:** The six common requirements and `status.hpp`; inherited by every
in-scope family. Platform adapters and verification runners are excluded.
**Level / purpose:** Unit/component regression plus explicitly identified build
and source inspection checks.
**Profiles:** [Shared profiles](profiles.md); numeric checks apply in both scalars.
**Setup:** Independent instances, distinguishable state/output sentinels, fresh
initialization for each case. Interleave two instances with distinct fixtures.
**Regression:** Common contract, scalar/backend, compiler/profile or public API
changes select these checks and affected families.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-C-FAILURE-STATUS | [C-FAILURE](../../requirements/common/failure.md#required-behavior) | Evaluate `succeeded` for each declared Status enumerator. | Only `success` is true; no reliance on numeric enum encoding. Detects inverted/overbroad success classification. | Native: [test_contracts.cpp](../../../../tests/common/test_contracts.cpp) |
| TC-C-FAILURE-FRAME | [C-FAILURE](../../requirements/common/failure.md#required-behavior) | Parameterized obligation: for each checked operation, use its listed failure partitions with distinguishable prior outputs and each legal alias. | Owning family specifies exact status and write set; all other declared outputs remain unchanged. Snapshot comparison detects partial publication. Do not duplicate family cases as extra coverage. | Delegated: family failure and transaction rows in the [design allocation](README.md#requirement-and-case-allocation); no duplicate generic pass |
| TC-C-INPUT-DOMAIN-PREMISES | [C-INPUT-DOMAIN](../../requirements/common/input-domain.md#required-behavior) | Audit each family's checked/assumed/ignored inputs; execute its paired-invalid and ignored-input cases. Include AHRS unconsumed translation fields and solve with small positive pivots. | Match owning contract, not blanket “invalid implies reject”; no invented condition cutoff, repair or physical sensor bound. Unsupported usage is a documented premise, not a rejection case. | Delegated: family consumed/ignored-input partitions and [profile review](profiles.md#acceptance-and-failure-frames); assumptions remain owned by each canonical contract |
| TC-C-NUMERICAL-PROFILE-MODES | [C-NUMERICAL-PROFILE](../../requirements/common/numerical-profile.md#required-behavior) | Inspect numeric limits, rounding/subnormal mode, compile flags and linked libm/backend for each numerical profile; execute its exact fixtures. | Correct binary32/64 and declared arithmetic profile. Unsupported FTZ/FMA/fast-math combinations block acceptance rather than reuse baseline evidence. Detects accidental compilation/profile drift. | Partial: [test_contracts.cpp](../../../../tests/common/test_contracts.cpp); host arithmetic probes; compiler flags and linked dependency identity require the build record |
| TC-C-NUMERICAL-ACCEPTANCE-CRITERIA | [C-NUMERICAL-ACCEPTANCE](../../requirements/common/numerical-acceptance.md#required-behavior) | Inspect every numerical assertion before accepting results; perturb a reference answer outside its allowed interval. | Domain, units, reference uncertainty and justified bound are present; bad result is rejected. Missing bounds remain visible, not an arbitrary epsilon or unconditional PASS. | Analysis: [numerical acceptance policy](profiles.md#acceptance-and-failure-frames); exact fixtures and negative controls are executable, while general domain/error budgets remain open in Partial family bindings |
| TC-C-LAYER-BOUNDARY-DEPENDENCIES | [C-LAYER-BOUNDARY](../../requirements/common/layer-boundaries.md#required-behavior) | Build numerical core and portable runtime consumers separately with only declared dependency targets; inspect transitive includes/symbols. | Core has no runtime/platform dependency; runtime accepts caller time/data without OS, uORB or replay coupling. Detects accidental layer inversion. | Partial: [test_compile_contracts.py](../../../../tests/common/test_compile_contracts.py); top-level public core/runtime headers compile independently and project include dependencies are checked; platform/OS absence requires source inspection |
| TC-C-CPP-SUBSET-RESTRICTIONS | [C-CPP-SUBSET](../../requirements/common/cpp-subset.md) | Compile supported public instantiations without exceptions/RTTI; inspect allocations, recursion, dynamic dispatch, synchronization, mutable globals and OS use. Instrument representative complete core/runtime sequences for heap use. | Restrictions hold within declared library boundary; dependency exceptions must be identified. A passing allocation sample alone cannot prove absence on all paths. | Partial: [test_compile_contracts.py](../../../../tests/common/test_compile_contracts.py); no-exception/no-RTTI compilation and representative object symbols; whole-source restrictions and runtime heap instrumentation remain separate |
| TC-C-CPP-SUBSET-BOUNDED | [C-CPP-SUBSET](../../requirements/common/cpp-subset.md) | Inspect loops/capacity-dependent work and collect compiler stack reports for each selected dimension/capacity instantiation, its maximum active length and nested transactions. | Work has a configuration-derived finite bound; reports identify nested stack needs. Generic templates have no single project-wide maximum. Deployment capacity, deadline and stack limits need explicit allocation before a budget PASS. | Analysis: [bounded C++ contract](../../requirements/common/cpp-subset.md); configuration-specific source/stack inspection and deployment budget allocation remain required, outside native pass counts |
| TC-C-CPP-SUBSET-INDEPENDENCE | [C-CPP-SUBSET](../../requirements/common/cpp-subset.md) | Run a deterministic valid/reject/recover sequence twice; interleave another independent estimator/runtime instance; reset and repeat. | Under identical profile and explicit input state, same outputs/events; other instance cannot alter result or history. Detects hidden shared state. | Partial: [test_contracts.cpp](../../../../tests/common/test_contracts.cpp); representative prediction and filter sequences; other owners are exercised by their family cases |
| TC-C-CPP-SUBSET-CONFIGURATION | C-CPP-SUBSET, [E-PCOV](../../requirements/core/covariance-prediction.md), [E-PRED-ATTITUDE](../../requirements/core/attitude-prediction.md), [E-RESET](../../requirements/core/covariance-reset.md), [E-MAG-TILT](../../requirements/core/magnetic-tilt.md) | Compile consumers with macros omitted, each valid 0/1 setting and each invalid value (-1, 2); inspect compile definitions across all application translation units. Compare omitted settings to explicit defaults. | Defaults codegen=1, quat_approx=0, reset_approx=0, mag_tilt=1; invalid values fail compilation. All translation units use one consistent configuration. ODR mismatch is a build-review error; no compiler diagnostic is promised for it. | Partial: [test_compile_contracts.py](../../../../tests/common/test_compile_contracts.py); defaults and valid/invalid macro compilation; application-wide translation-unit consistency remains build inspection |

## Source allocation

`include/formal_eskf/status.hpp`: Status and succeeded → STATUS.
The six common requirements are allocated above; their operation-specific
conditions are instantiated in family tables, not replaced by a single generic
test. Static restrictions and bounds need inspection/analysis even when native
tests pass.

## Gaps and exclusions

Deployment WCET, total task stack, scheduler behavior, compiler qualification,
external-library internals and target numerical acceptance are not established
by these host checks. Profile-specific budgets and currently unspecified
numerical bounds prevent claims beyond the declared exact fixtures/domains.
No percentage of whole-program correctness follows from this allocation.

## Review

Pending owner review. Design review is separate from requirement approval and
execution evidence.
