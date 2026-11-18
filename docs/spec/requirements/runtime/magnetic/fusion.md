# R-MAGNETOMETER-FUSION: field checks and gated magnetic correction

**ID:** `R-MAGNETOMETER-FUSION`

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

**Uses:** [R-MAGNETIC-FIELD](field.md), [E-OBS-MAGNETIC](../../core/magnetic-observation.md), [E-INNOV](../../core/innovation.md).

## Required behavior

`runtime::fuse_magnetometer` has explicit AHRS/INS overloads sharing field qualification and residual arithmetic, and returns `MagnetometerFusionResult` with separate `field` and `fusion` diagnostics. `fusion.decision/status` describe the overall call. It retains the existing core's calibration, time, unit-quaternion, PSD/SPD, noise-independence and whole-object alias premises.

- Check finite nominal fields, then apply `R-MAGNETIC-FIELD` to this prior. The optional health-reference pointer forwards independent health geometry; only `m_n` reaches the innovation, correction and attitude protection. Supplying the observation reference for both roles retains the same correction behavior. A field error returns `failed` with its status; a field rejection returns `rejected` with `success`. Neither runs the innovation gate or correction. Innovation diagnostics remain invalid; rejection does not certify unused covariance or later-stage parameters.
- After field acceptance, form the existing magnetic residual `r=m_b-R(q_nb)^T m_n` and `magnetometer_jacobian(state,m_n)`. Gate using `diag(H P H^T + V)` through the existing same-prior gate stage. Any component ratio above one rejects the entire three-axis group. The gate uses the full observation model, not a projected H or measurement-noise-only variance, in both tilt configurations.
- After gate acceptance, call `correct_magnetometer` with identical inputs, full V, normalization threshold and INS bias permissions. Do not call generic correction and bypass `ESKF_MAG_TILT`. The existing projection, constrained-gain Joseph update and injection/reset remain unchanged.
- Only `fusion.decision=fused` publishes state and covariance. Rejection/failure preserves both outputs, including all four independent whole-prior alias arrangements. Field diagnostics remain valid if a later stage fails; innovation diagnostics remain valid after gate success even if correction subsequently fails. Before the relevant stage succeeds, its diagnostics are invalid placeholders. Preserve the downstream failure status.

This is one stateless observation transaction, not full sensor health qualification, fusion lifecycle, persistent-rejection recovery, yaw initialization or adapter wiring. Native tests cover both configurations and scalar formats, full correlated covariance, all 64 INS bias masks, legal aliases and stage failures. ESBMC checks the shared field-first observation preparation in both scalar formats: the original health-reference selection and policy, field diagnostics, stop priority, exact measured-minus-predicted residual, finite-result checks and atomic disjoint scratch outputs. Same-type field qualification and inverse-action input-frame producers close this helper boundary. The complete public AHRS/INS transaction is checked compositionally in binary32/64, with all four independent state/covariance alias arrangements and all four optional health-reference arrangements. Actual finite-state validation, attitude Jacobian construction, stage ordering, exact diagnostic propagation, INS bias permissions and atomic publication consume the same-type field/observation, measurement-check and magnetic-correction producer contracts. Arbitrary producer statuses/results include failures; no successful, finite, unit or PSD input is assumed. The wrapper is independent of the tilt compile option and composes with both separately proved M0/M1 correction families. Module/replay sequencing remains outside this portable transaction claim. No new mathematical or target-libm accuracy claim follows.










## Caller bindings

- Replay magnetic observations: [magnetic-observation.md](../../integration/replay/magnetic-observation.md).
- Replay magnetic field checks: [magnetic-field.md](../../integration/replay/magnetic-field.md).
- Replay geographic magnetic model: [magnetic-model.md](../../integration/replay/magnetic-model.md).
- Replay magnetic lifecycle: [magnetic-lifecycle.md](../../integration/replay/magnetic-lifecycle.md).
