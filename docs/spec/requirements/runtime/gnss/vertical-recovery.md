# R-GNSS-VERTICAL-RECOVERY: bounded vertical innovations during IMU faults

**ID:** `R-GNSS-VERTICAL-RECOVERY`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [E-INNOV](../../core/innovation.md).

## Required behavior

`runtime/navigation.hpp` provides GNSS velocity fusion and
source-independent NED Down position fusion (`fuse_height`). Generic gates and
ungated correction APIs are unchanged. The caller must first establish source
quality/lifecycle permission, frame/time, unit quaternion, symmetric PSD P and
SPD measurement noise, as for ordinary
fusion. A fault never grants source permission or requests a reset.

| Observation | Additional condition to release a rejected gate |
|---|---|
| 3D GNSS velocity | Bad vertical acceleration, Down ratio > 1, both N/E ratios strictly < 1, receiver speed accuracy strictly below its configured maximum |
| Down position (GNSS; also qualified barometer in the module) | Bad vertical acceleration and rejected height gate |

Only the vertical innovation is constrained to `[-gate_sigma * sqrt(S_D),
+gate_sigma * sqrt(S_D)]`. Preserve its sign, horizontal residuals, H, V and
bias-update permissions; call the existing checked correction with that selected
residual. Do not re-gate the rounded boundary value. Ordinary gate equality is
accepted, but horizontal equality prevents the velocity exception. Receiver
accuracy is its reported standard deviation in m/s, not the noise-floor-adjusted
variance; its strict exception bound applies even when the general quality
speed-accuracy check is disabled. Both accuracy values must be finite, accuracy
nonnegative and maximum positive, regardless of the fault flag.

Height fusion additionally accepts a finite nonnegative `deadzone` in metres,
zero by default. After the IMU-fault limit, transform a positive selected residual
`r` to `max(r - deadzone, 0)`; retain zero/negative residuals. Never recompute or
release the gate because of this transform. Nonfinite/negative settings fail
before gate/correction and preserve outputs. `R-HEIGHT-FUSION` defines when the
module may supply a nonzero value. `innovation_limited` continues to describe
only the acceleration-fault clamp; `fusion_innovation` includes both transforms.

`NavigationFusionResult` retains raw innovation, S and test ratio in `fusion`, and
separately exposes `fusion_innovation` and `innovation_limited`. Raw ratio > 1
can therefore accompany successful limited fusion. Selected innovation is valid
only with `diagnostics_valid`; only `decision == fused` means it was applied.
`check_gnss_velocity` returns `NavigationInnovationCheck` without correction, for an
active source's quality-rejected sample. Its `gate` member owns status, diagnostic
validity and effective gate rejection; it cannot report successful fusion.
Gate/numerical errors retain their existing statuses; invalid accuracy returns
non_finite_input/domain_error. Rejection or any failure, including
LLT/normalization after limiting, preserves both outputs.
Independent whole-prior state/P aliases are supported; other storage is disjoint.

Behavior reference: PX4 v1.16.2
[velocity exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L209)
and [height exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp#L36).
Alignment covers predicates, limiting and fault-consumption order. Our grouped
Joseph correction and latest-prior diagnostics differ from PX4's scalar update
schedule; replay cadence/batching and missing-data safeguards also differ.
Native backend and replay tests are evidence for these contracts, not flight
accuracy or full EKF2 equivalence. `NavigationRecovery.lean` proves exact-real
clipping bounds, sign, idempotence, selected-axis isolation and the resulting
ratio bound. This is not a rounded-boundary re-gating guarantee.

`navigation_recovery.cpp` checks actual gate diagnostic production, the actual
limiter and both public recovery callers in binary32/64 with all independent
state/P alias modes. The limiter retains raw diagnostics, uses the selected
variance for its root request and handles arbitrary scalar-library returns.
The public callers preserve strict policy boundaries and publish only after
successful correction. The height caller's symbolic deadzone covers disabled,
positive and invalid values, ordering after the limit, unchanged raw gate evidence
and all independent state/P alias modes. Same-type covariance/correction contracts and the
separately checked exact limiter relation bound the composed caller proof;
they are not proofs of external backend internals. Shared IEEE scalar lemmas
establish nonnegative finite limits and sign/bounds under an explicit finite,
nonnegative-root premise. Root accuracy, machine-checked real-to-C++ refinement,
external source permission and actual module/replay health ordering remain
separate obligations.

## Caller bindings

- [R-REPLAY-GNSS-GATING](../../integration/replay/gnss-gating.md).
- [R-HEIGHT-FUSION](../../integration/px4/height-fusion.md).
