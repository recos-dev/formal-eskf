# Shared Test Profiles and Oracles

**Status:** Draft

This is the shared setup for the [core/runtime test design](README.md), under
the [test standard](../testing.md). Existing test implementations and their
tolerances are not acceptance criteria. Bindings distinguish **Native** execution,
**Partial** acceptance, **Analysis** and **Delegated** obligations; an unresolved
**Pending** binding is a registration gap, never a passing case.

## Scope and configurations

| Dimension | Required selection and reduction rule |
|---|---|
| Estimator | INS (16 nominal coefficients, 15 errors) and rotation-only AHRS (4 coefficients, 3 errors). INS-only APIs are excluded from AHRS, not counted as passes. |
| Scalar | Binary32 and binary64 for numerical cases. Integer/Boolean policy cases need one execution per distinct implementation, with numerical threshold calculations repeated for both formats where used. |
| Backend | Run public arithmetic/conformance and numerical compositions with Eigen, PX4 matrix and CMSIS-DSP adapters when dependencies are available. Unavailable supported adapters are **Blocked**, not silently omitted. Test the abstraction, not every external library function. A simple independent host reference supplies the oracle, not another production backend alone. |
| Generated arithmetic | `ESKF_CODEGEN=0/1` for INS covariance, innovation, Joseph correction and reset. Compare each to the same independent mathematical oracle. Agreement between two implementations is supporting evidence. AHRS has no generated arithmetic claim. |
| Attitude prediction | `ESKF_QUAT_APPROX=0/1` for nominal and covariance prediction and their combined step. These are different models at nonzero dt; do not demand identical outputs. |
| Reset | `ESKF_RESET_APPROX=0/1` for reset and injection/correction transactions. Check each selected equation; compare approximation error only on a specified small-angle domain. |
| Magnetic update | `ESKF_MAG_TILT=0/1` for magnetic gain/feedback and connected runtime fusion. Inspect the projected or full gain and its Joseph covariance consistently. |
| Build | Run behavioral checks in Debug and optimized Release with active assertions; GCC and Clang support builds. Record actual versions, optimization, floating-point options and dependency revisions. |
| Floating point | Default round-to-nearest, ties-to-even, no fast-math/reassociation, FMA contraction disabled under [C-NUMERICAL-PROFILE](../../requirements/common/numerical-profile.md). Record subnormal handling. FTZ/DAZ or FMA-enabled target profiles need separate acceptance; a host pass cannot establish them. |
| Target | Host results cover host execution. Target stack, timing, instruction set, libm and resource budgets require target evidence; integration and deployment tests are outside this design. |

Avoid a blind Cartesian product. Primitive tests have no estimator mode macros.
Prediction tests vary codegen × attitude; reset tests codegen × reset;
magnetic correction tests codegen × tilt × reset where the complete transaction
uses all three. The core sequence case exercises all 16 macro combinations for
INS, on the primary host backend in both scalars, to expose composition effects.
Run each other backend's affected compositions in the default and each relevant
alternate path; test all combinations touching a backend-specific dispatch.
Document any further profile reduction before claiming acceptance. Behavioral
policy-only cases do not need repeats for unrelated macros.

## Fixtures and expected values

Each family names its fixture recipe. Implementation shall materialize exact
values/seed, parameter values, order of operations under test and observations
in a selectable case. A named partition is not executable coverage until its
members are explicit and individually reported. There is no target case-count
quota and no automatic promotion of old tests to this design.

| Oracle | Appropriate use and limitation |
|---|---|
| Hand-calculated exact fixture | Dyadic coefficients, basis vectors, signed permutations, small integer matrices and scalar state transitions. Confirm intermediates are exactly representable before using exact numerical equality. Useful for signs, offsets, frames, dt powers and forbidden writes. |
| Independent numerical reference | Compute the requirement equations with arbitrary precision or interval bounds, independently of production helpers/generated expressions. Double is not automatically an adequate reference for binary64. Include reference error in the acceptance budget. |
| Residual/metamorphic check | Solve residual, SO(3) action/composition, symmetry, frame invariance, zero innovation, permutation or scale relations. Pair with a known-answer case: two mutually wrong implementations can satisfy a relation. Finite sampling is not a universal proof. |
| State-transition model | A small declarative table or host model of timestamps, counters, flags and selected payloads. Evaluate every intermediate observation. Derive it from the requirement, not by copying production branches. |
| Failure control | A documented substitute at an existing backend/scalar seam may return selected statuses/results to expose otherwise rare propagation failures. Call the actual public production function. Record the substitute and its permitted contract; this proves caller response, not that the deployed producer can generate that value. |
| Source/build inspection | Compile-time rejection, dependency/language restrictions, bounded loops, no explicit inverse and one factorization per solve. Use inspection or passive instrumentation without changing the arithmetic/control result. These are not numerical runtime assertions. |

Use independent physical scenarios and derivatives for observation Jacobians;
do not call the production Jacobian to compute its own expected result. Numerical
differentiation must sweep step size, account for truncation/reference errors and
use right-local attitude perturbations. It supports, but does not replace, exact
symbolic correspondence.

## Acceptance and failure frames

- Status, selected source, timestamp/index, event order, permissions and exact
  no-update conditions use their specified equality. For preserved floating
  coefficients check representations where required, including signed zero;
  exclude padding and unspecified NaN payload preservation. Do not compare
  uninitialized storage.
- Snapshot every public output and relevant owner state before rejected calls.
  Check allowed mutations separately from preserved fields. Runtime diagnostic
  history or revoked fusion permissions may change on rejection; estimator
  failure atomicity does not imply that the whole runtime object rolls back.
- Test disjoint outputs and each permitted whole-object alias; do not invent
  partial-storage aliases or use invalid indices. Make sentinels distinguish
  every field/coefficient so a wrong offset cannot pass unnoticed.
- Derive invalid-input priority from the owning requirement. Pair failures at
  adjacent validation stages and verify short-circuit effects; do not multiply
  every invalid value by every other invalid value without a fault rationale.
- An independent numerical case needs an input/conditioning domain, units,
  reference uncertainty and absolute/relative or residual bounds as required by
  [C-NUMERICAL-ACCEPTANCE](../../requirements/common/numerical-acceptance.md).
  Where these are not specified, the case remains designed but **blocked for
  numerical acceptance**. Do not choose epsilon by observing production error.
- For threshold tests use below/equal/above represented values. Validate the
  computed quantity actually lands on the desired side after rounding. Do not
  assume `sqrt(nextafter(t))` squares back to the intended threshold.
- Construct public values through supported APIs. Private representation hacks
  do not provide legitimate arbitrary-quaternion test inputs. Use reachable
  sequences, a reviewed existing seam, or scoped source proof; retain a gap
  where the desired domain has no valid native construction route.

## Regression and sensitivity

Run a changed family's cases plus callers consuming its result/contract; retain
counterexamples as regression cases with stable IDs. Run the full supported
core/runtime selection before a release. Resource-intensive numerical campaigns
may be separate from fast exact/state cases, with their scope reported explicitly.

For each family, challenge the oracle with at least one relevant controlled
defect: wrong sign/transpose/index, skipped validation, premature publication,
off-by-one threshold, stale-source reuse or wrong configuration dispatch.
Record which assertion detects it. Mutation tools are optional; a reviewed
negative control is sufficient. This is planned sensitivity validation, not a
claim that mutations or cases have already run. Do not change production code
or weaken the expected result to obtain a green test.

The case tables are the design allocation. Implementation maps them to executable
selectors and fixtures, then reconciles statement/branch coverage to find missed
behavior. Structural coverage is supporting evidence, not the design denominator
or a substitute for an independent oracle.
