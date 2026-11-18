# R-MEASUREMENT-FUSION: gated correction of one observation group

**ID:** `R-MEASUREMENT-FUSION`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [E-CORRECT](../core/correction.md), [E-INNOV](../core/innovation.md), [E-OBS](../core/observations.md).

## Required behavior

`runtime::fuse_measurement` supports INS/AHRS and accepts `r = z - h(state)`, `H`, full measurement covariance `V`, an explicit sigma threshold and the existing normalization threshold. The caller supplies one prior, zero error mean, valid frames/time, unit quaternion, symmetric PSD `P` and symmetric SPD `V`, as for `correct`. INS additionally accepts the same optional per-axis bias permissions.

`runtime::check_measurement` evaluates the same gate without attempting correction. Its `MeasurementCheck` reports status, diagnostic validity and rejection; `passed()` requires successful, valid, non-rejected evidence. It owns no history and grants neither source permission nor permission to reuse a prior linearization after the estimate changes.

- Check finite nominal fields and sigma, then call the numerical core's `correction_covariances`. This operation checks the correction inputs in their existing order and publishes `PHt` and `S` together only on success; its outputs are disjoint from each other and from inputs. It does not inspect nominal state, solve for gain or perform injection. Compute `PHt = P * transpose(H)`, then `S = H * PHt + V`, with the same finite checks and symmetry cleanup as correction. Gate with `r` and `diag(S)`, never `diag(V)`. Rejection is componentwise, not a joint NIS test; gate errors retain `E-INNOV` statuses.
- Return a fresh `FusionResult` with `decision = fused`, `rejected` or `failed`. Only `fused` publishes both state and covariance. `rejected` has `status = success`; any numerical/input failure has `decision = failed` and its failing status. Neither rejection nor failure modifies either output. Each complete output may independently alias its respective prior; all other input/output storage is disjoint.
- Only after the gate succeeds, publish `innovation`, `innovation_variance` and `test_ratio` with `diagnostics_valid = true`, including rejected observations. These diagnostics describe the call's prior and remain valid if correction later fails. Before gate success, diagnostics are invalid zero placeholders; never reuse a prior call's diagnostics.
- If any ratio exceeds one, reject this whole observation group without correction, LLT or injection. Otherwise call the existing correction with the identical state, `P`, `r`, `H`, `V`, normalization threshold and bias permissions. The gate and correction use the same covariance implementation. Accepted calls intentionally recompute the products: correction remains independently checked, without a retained workspace or caller-supplied cached gain. GNSS may use a limited residual, and magnetic correction retains its own projection/check order. Equations and validation order are unchanged. A gate pass does not certify full covariance validity, conditioning or prior quaternion norm. Rejection does not certify estimator health.

The named runtime APIs form their residual from the supplied state and reuse the existing core Jacobians:

| API | One rejection group |
|---|---|
| `fuse_horizontal_position` | NED north/east position |
| `fuse_vertical_position` | NED down position, not altitude or HAGL |
| `fuse_horizontal_velocity` | NED north/east velocity |
| `fuse_velocity` | All three NED velocity components |

These models preserve the corresponding core API's units, full within-group noise correlations and bias permissions. Non-finite observations return `non_finite_input`; residual overflow from finite observations/state returns `non_finite_result`. Use horizontal OR 3D velocity for a given sample, not both.

Each call is independent: a rejected or failed group neither undoes previous fusion nor automatically rejects later groups. After a successful group, form the next residual/Jacobian and gate using the newly committed state/covariance; after rejection/failure retain the prior. No fixed sensor order or whole-GNSS transaction is imposed. Separate groups require uncorrelated measurement noise across groups, not zero state cross-covariance; use a joint model if cross-group noise correlations must be retained. Alignment, GNSS quality, source lifecycle, persistent-rejection recovery, clocks and last-fusion timestamps remain caller/runtime work.

Native tests cover INS/AHRS, binary32/binary64, legal aliases, grouped rejection, input/arithmetic and downstream failures, bias permissions and a cross-coupled sequential-fusion case. They do not establish flight suitability. The `fusion` ESBMC suite checks the public covariance-preparation frame, actual portable check/fuse control flow and these four named wrappers: binary32/binary64, INS/AHRS where applicable, observation sizes 1/2/3, every legal state/covariance alias and symbolic INS bias permissions. It composes with the same-backend `E-INNOV`, correction and injection/reset producers; arbitrary inputs and failing intermediate results remain admitted. The source assertions cover skipped correction on rejection, diagnostics retained on downstream failure, unchanged inputs and atomic publication. Existing Lean gate/correction theorems are mathematical dependencies, not a new proof of the runtime state machine or a machine-checked Lean-to-C++ refinement. [Replay GNSS settings/wiring](../integration/replay/gnss-gating.md), caller history/timing, external backends and numerical error bounds remain open. See the [verification scope and evidence rules](../../../guides/verification.md).

## Caller bindings

- Replay GNSS gating: [R-REPLAY-GNSS-GATING](../integration/replay/gnss-gating.md).
