# Core and Runtime Test Design

**Status:** Draft

This defines the tests the numerical core and portable runtime need, derived
from requirements and production responsibilities. Existing tests are treated
as absent during design. They may later satisfy a case only after their inputs,
oracle, assertions and profiles have been reviewed against it. This is a design
baseline for review, not a coverage report or approval of current code.

Use the [test standard](../testing.md), then [shared profiles and oracles](profiles.md)
and the relevant family below. A case table records specific partitions/actions,
expected outputs/history, an independent oracle and the defect it should expose.
All executable bindings are Pending. No new test or production implementation is
part of this draft.

## Scope inventory

| Layer | Canonical requirements | Production headers | Responsibility |
|---|---:|---:|---|
| Common | 6 | 1 | Failure/domain/numerical/language/layer contracts; Status. |
| Math | 16 | 15 | Scalar, fixed-size linalg/adapters, quaternion and SO(3). |
| ESKF core | 21 | 21 | State, prediction, correction, observations and transactions, including generated kernels. |
| Portable runtime | 22 | 13 | Admission, health, lifecycle, reset/fusion orchestration and fixed-capacity storage. |
| Software capabilities | 5 | Shared above | SW-CORE, SW-GNSS, SW-GRAVITY, SW-IMU and SW-MAGNETIC compositions. |
| **Total** | **70** | **50** | Inventory denominator, not proof/test completion. |

The 70 IDs include parent and child requirement documents. They are not 70
independent passing claims, and parent/child evidence must not be added together
to manufacture a completion percentage.

The header boundary is `include/formal_eskf/{eskf,runtime,linalg,scalar,so3}` plus
`status.hpp`. Each family's source/operation table maps public API groups and
internal dependencies to cases or an explicit disposition. One header is not one
test, and this manual allocation is not an AST function-by-function coverage claim.

Excluded: `integrations/`, `examples/`, PX4/replay owners, uORB, geodesy, actual
flight scheduling, tool runners and the generator pipeline. In particular,
ImuHistory, ObservationHistory, OutputPredictor and HeightAiding currently owned
under the PX4 integration are outside this task even where proof work groups
them with portable runtime. Portable sequences calling real core/runtime APIs
remain in scope; this exclusion does not remove component-composition tests.
External linalg internals and deployment qualification have separate boundaries.

## Requirement and case allocation

Each listed ID is owned by the linked family for this design. Common contracts
apply to all families. Parent composition cases are additional obligations; a
child list does not establish the parent's behavior. Detailed case rows identify
named clauses, partitions, profiles and unresolved criteria without duplicating
the canonical equations.

| Family | Primary requirement allocation |
|---|---|
| [Shared contracts](common.md) | C-FAILURE, C-INPUT-DOMAIN, C-NUMERICAL-PROFILE, C-NUMERICAL-ACCEPTANCE, C-LAYER-BOUNDARY, C-CPP-SUBSET. |
| [Scalar and linalg](math/scalar-linalg.md) | F-SCALAR, F-LINALG, F-SOLVE. |
| [Quaternion and SO(3)](math/quaternion-so3.md) | Q-IDENTITY, Q-MUL, Q-INVERSE, Q-NORMALIZE, Q-ROT-COMPOSE, Q-SIGN, Q-ROT-MATRIX, Q-ROTATE, SO3-HAT, SO3-EXP, SO3-LOG, SO3-SMALL-ANGLE, F-JR. |
| [State and propagation](core/propagation.md) | E-STATE, E-PRED, E-PRED-ATTITUDE, E-PRED-INS, E-NOISE, E-PCOV, E-STEP-PRED. |
| [Correction, injection and reset](core/update.md) | E-CORRECT, E-BIAS-UPDATE, E-INJECT, E-RESET, E-STEP-INJECT-RESET, E-STEP-CORRECT, E-STEP; SW-CORE. |
| [Observations and innovation](core/observations.md) | E-OBS, E-OBS-POSITION, E-OBS-VELOCITY, E-OBS-ACCEL, E-OBS-MAGNETIC, E-MAG-TILT, E-INNOV. |
| [GNSS and navigation](runtime/gnss-navigation.md) | R-MEASUREMENT-FUSION, R-GNSS-QUALITY, R-GNSS-FUSION, R-GNSS-RESET, R-GNSS-VERTICAL-RECOVERY; SW-GNSS. |
| [Gravity and IMU](runtime/gravity-imu.md) | R-ACCELEROMETER-MAGNITUDE, R-ACCELEROMETER-SAMPLING, R-ACCELEROMETER-CONDITIONS, R-ACCELEROMETER-FUSION, R-IMU-BIAS-INHIBITION, R-IMU-PROCESS-NOISE, R-BIAS-PROCESS-NOISE, R-BIAS-LIMITS, R-VERTICAL-ACCELERATION; SW-GRAVITY, SW-IMU. |
| [Magnetic aiding](runtime/magnetic.md) | R-MAGNETIC-REFERENCE, R-MAGNETIC-FIELD, R-MAGNETIC-HEADING, R-MAGNETOMETER-FILTER, R-MAGNETOMETER-FUSION, R-MAGNETOMETER-RESET, R-MAGNETOMETER-LIFECYCLE; SW-MAGNETIC. |
| [Storage and alignment](runtime/storage-alignment.md) | R-RING-BUFFER; TiltAlignment candidate cases with incomplete ownership under R-ACCELEROMETER-CONDITIONS, not a new approved requirement. |

## What completeness means here

The review proceeds in both directions:

1. Requirement → consumed/assumed/ignored inputs, equations, success/failure
   effects, thresholds, event transitions and applicable configurations → cases
   or a justified alternate verification method/open criterion.
2. Production header/API group → behavioral owner and case allocation; unowned
   behavior remains a requirement gap. Source behavior cannot silently become a
   normative oracle.
3. Case → meaningful defect detection, independently justified expected result,
   reproducible fixture, profile selection, and all specified observations.
4. Capability → sequences of real operations with coherent priors, source/time
   identities and feedback; passing primitives alone cannot close composition.

No statement/branch/MC/DC percentage is claimed. The finite cases do not prove
all real inputs, arbitrarily long event sequences or all template instantiations.
Formal invariants and source proofs remain complementary evidence, with matching
assumptions and profiles. A substitute backend tests the caller's contract use;
it does not establish the actual producer's correctness.

Case counts refer to table rows, not assertions or executable invocations. A row
may require several input partitions and case/profile executions. The current
design allocates every in-scope requirement file and header group, while explicitly
retaining the gaps below. Existing runs contribute **no accepted execution
coverage** to this clean-slate design until mapped and reviewed.

## Decisions needed before acceptance

| Gap | Required action |
|---|---|
| Numerical criteria | Define bounded input/conditioning domains and error budgets for general libm, reductions, SO(3), covariance, solve, filter and repeated-operation cases. Exact fixtures/status/state-machine cases can proceed independently. |
| Derived linalg contracts | Clarify canonical normalization, symmetry and nonfinite max_abs behavior identified in the math family; proposed source bindings remain Draft. |
| TiltAlignment ownership | Extract/clarify its behavior currently partly described in gravity-condition evidence prose, including construction/reconstruction, accessors and input premises. Keep the candidate binding under R-ACCELEROMETER-CONDITIONS unapproved; it is not accepted normative coverage. |
| Publicly unreachable failure inputs | Select a legitimate seam or scoped source proof for malformed private quaternion states and controlled producer failures. Do not fabricate objects through layout casts. |
| Observation of call order | Select passive instrumentation/source evidence where output equality cannot prove a callee was skipped or which arguments it consumed. Actual body tests remain required. |
| Supported instance/target limits | Bind capacities, generic sizes, FTZ/FMA behavior, dependencies and resource budgets to explicit profiles. Non-10000-us AccelerometerSample instantiations need an extended cadence contract. No universal maximum or host-to-target acceptance is implied. |
| Execution registration | Bind each approved case/partition/profile to selectors, fixtures, retrievable results and freshness checks in V-02; implementation and acceptance are still pending. |

These decisions are not waivers. A case with an unresolved oracle/budget cannot
be reported as passing that property. Requirements remain Draft until the owner
approves their revision; a mechanically complete allocation does not approve them.

## Implementation order after design review

1. Review the common/profile rules and one complete core transaction family.
   Resolve its domain/fixture/bound questions; establish selectable per-case
   evidence with a small exact/status subset before broad tooling expansion.
2. Implement or qualify primitives and their callers in dependency order:
   scalar/linalg → SO(3) → propagation → correction/reset → observations/gate.
   Add full transactions alongside their dependencies, not only isolated units.
3. Implement runtime state-machine cases and real portable compositions:
   storage/alignment → IMU/gravity → GNSS/navigation → magnetic aiding.
   Independent families may proceed in parallel after shared criteria review.
4. Compare existing tests against this design. Reuse adequate cases, strengthen
   weak or circular oracles, remove redundant cases only after their obligations
   have a reviewed replacement, and retain meaningful defect regressions.
5. Reconcile executable case/profile coverage and measured structural gaps;
   validate negative controls and evidence completeness before accepting a baseline.

This elaborates [V-02](../../../roadmap.md#v-02--design-and-execution-evidence-pilot).
It does not reorder V-01 proofs or bring integration tests into the current scope.
Existing tests will be surveyed during implementation; they are not the source
of this design or a second authoritative allocation.

## Review

Pending owner review. Independent core/runtime design and cross-checking can
identify omissions; they do not establish execution coverage or approval.
