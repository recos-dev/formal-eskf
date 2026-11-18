# PX4-OWN: ownership and interfaces

**ID:** `PX4-OWN`

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

| Responsibility | Owner | Boundary |
|---|---|---|
| Module lifecycle and platform access | PX4 adapter | Work queue, uORB, platform clock, parameter binding and publication |
| Estimation session | One estimator owner, executed by that work item | Initialization/alignment, nominal state, covariance, source policies, fusion order and fault state |
| Time alignment | Estimation session | IMU accumulation, observation/context histories and delayed fusion horizon |
| Current-time output | Output predictor within the same session | Its own propagated output state/history, corrected by the main filter; no independent covariance or fusion decisions |
| Numerical operations | Existing ESKF/SO(3)/linalg/scalar core | Caller-supplied values and explicit results; no platform effects |
| Diagnostics | Snapshot produced by the owner | Readers consume a coherent snapshot, not live mutable estimator fields |

There is exactly one writer of estimator and output-predictor state. Aiding
components own their source history, not copies of the full filter. Alignment,
reset bookkeeping and estimator faults belong to the session, not to one sensor.

These are responsibility boundaries, not a demand for one class per row.
Use concrete state and direct calls; do not add a generic sensor framework,
virtual policies, callbacks between numerical stages or forwarding-only facades.
PX4 messages are converted at the adapter boundary. Estimation operations receive
explicit values, time and context and can be exercised without a work queue.
