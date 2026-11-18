# PX4-VERIFY: acceptance and change control

**ID:** `PX4-VERIFY`

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

| Contract | Required acceptance evidence |
|---|---|
| Ownership and lifecycle | Production-path tests for initialization, stop, failure, concurrent diagnostics and lifetime safety |
| Timing and execution | Deterministic timestamped tests for rate conversion, delay/jitter, selection, duplicates, overflow, missing IMU and context alignment |
| Output and resets | Independent propagation/tracking cases; coherent sample times, reset deltas/counters, validity and uncertainty horizons |
| Real-time deployment | Target build/dependency inspection and resource/deadline evidence under the declared worst-load scenarios |
| Numerical composition | Existing core requirements plus affected runtime/module obligations, linked to the actual production paths |

Use these stable `PX4-*` design clauses when recording implementation and review
evidence. They are not additional passing rows in the current agent-review map.
Map each implemented scope to its requirements, source/configuration and retained
test/proof results before claiming completion. Lean/ESBMC component results do
not prove PX4 scheduling, dependency behavior or target WCET. The
[traceability criteria](../../../formal-verification-traceability-and-review-criteria.md)
continue to govern evidence and certification-related claims.

Behavior-preserving refactors shall preserve the adopted contracts. Changes to
sampling, fusion timing, output prediction or publication order require reviewed
behavioral requirements and new expected-result evidence; they are not editorial
or refactor-only changes. Target resource acceptance remains a separate obligation.

Changes to this architecture require a reviewed specification change and impact
assessment for affected contracts/evidence. Keep implementation status in
[FEATURE_TODO.md](../../../../../FEATURE_TODO.md) and [ROADMAP.md](../../../../../ROADMAP.md), not
as a second changing backlog here.
