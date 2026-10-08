# Magnetometer Tilt Protection

Implemented for AHRS and INS with compile-time `ESKF_MAG_TILT=1` by default.
Set it to `0` to retain the original full-field correction. Protection restricts
the attitude correction to rotation about navigation Down; it is not magnetic-disturbance detection or a
complete heading-fusion policy. The new projection has native tests, but its
Lean/ESBMC evidence and floating-point tilt-error bound remain pending.

## Current behavior

The [magnetometer model](../include/formal_eskf/eskf/magnetometer.hpp) uses a fixed
navigation-frame reference field and one three-axis batch correction:

```text
h = R(q_nb)^T * m_n
r = m_b - h
H_theta = hat(h)
```

With `ESKF_MAG_TILT=0`, magnetic observations can change both heading and tilt.
The default `ESKF_MAG_TILT=1` keeps this observation model and constrains only the
attitude gain. INS position, velocity and permitted biases can still change
through cross-covariances. The named magnetometer correction API has no built-in
innovation gate, disturbance checks or magnetic-bias state.

There is no runtime mode argument or selection branch. Existing call signatures
are unchanged, but their default behavior now protects tilt. CMake propagates
the option through `formal_eskf::core`:

```bash
cmake -S . -B build -DESKF_MAG_TILT=ON   # Default: navigation-Z projection.
# Use -DESKF_MAG_TILT=OFF for unrestricted full-field correction.
```

Non-CMake consumers can pass `-DESKF_MAG_TILT=0` or `1`; the header defaults to
`1` and rejects other values at compile time. Use one consistent setting across
all translation units to avoid incompatible inline/template definitions. With
protection disabled, the projection path is not compiled; when enabled, its
arithmetic still has a cost. This removes runtime policy selection, not the cost
of performing the selected algorithm.

```cpp
// AHRS
try_correct_magnetometer(state, P, m_b, m_n, V, minimum_norm,
                         state_output, P_output);

// INS: retain the existing per-axis bias permissions.
try_correct_magnetometer(state, P, m_b, m_n, V, minimum_norm,
                         state_output, P_output, bias_update);
```

Omit `bias_update` or use `{}` to enable all six bias updates. This permission
control is independent of the compile-time magnetic tilt policy.

## What the projection means

For a unit column vector `u`, `u * u^T` keeps only the component along `u`.
Here `T` means transpose, not differentiation. Apply it to a vector `v` in two
steps:

```text
s      = u^T * v          # Dot product: signed amount along u.
v_keep = u * s            # Put that amount back along u.
       = (u * u^T) * v
```

For example:

```text
u = [0, 0, 1]^T
v = [1, 2, 3]^T
s = 3
v_keep = [0, 0, 3]^T
```

This removes the X/Y components. But projection is not always axis selection:
if `u = [1/sqrt(2), 0, 1/sqrt(2)]^T`, the same `v` projects to `[2, 0, 2]^T`.
The retained direction is tilted relative to the coordinate axes.

For unit `u`, `u^T * u = 1`, while `u * u^T` is a 3x3 matrix, not the scalar one
or the identity matrix. Multiplication order matters.

## Projection in our attitude coordinates

The [state conventions](spec/eskf-state-conventions.md) use body-to-NED rotation
and a local, right-multiplicative attitude error:

```text
q_true = q_nb * Exp_q(delta_theta_b)
```

Let `e_D` be the NED Down unit vector. Express this navigation vertical direction
in the body frame, then project each column of the attitude gain onto it:

```text
R        = R(q_nb)
e_D      = [0, 0, 1]^T
u_b      = R^T * e_D
Pi_b     = u_b * u_b^T
K_theta' = Pi_b * K_theta
```

`K_theta` contains the three attitude-error rows of the gain, not four quaternion
coefficient rows. With three magnetic observations, it is a 3x3 block.

The resulting attitude correction has only one direction:

```text
delta_theta_b = K_theta' * r
              = u_b * (u_b^T * K_theta * r)
              = u_b * alpha
```

For an exact unit quaternion, right multiplication about `u_b` equals left
multiplication about navigation Down:

```text
R_new = R * Exp_SO3(u_b * alpha)
      = Exp_SO3(e_D * alpha) * R

R_new^T * e_D = R^T * e_D
```

The last equality expresses tilt preservation: the navigation vertical direction
seen from the body does not change. This avoids relying on Euler angles near
their singularities. Directly clearing the body X/Y gain rows would instead
retain rotation about body Z, which generally changes tilt when the vehicle is
already tilted. Down means local NED vertical, not the Earth's ECEF rotation
axis. These are exact geometric statements, not certified floating-point bounds.

## Relationship to PX4

Reference: PX4 v1.16.2, commit `54f0455ffcd755534539a7cf33a09a20bf71d29d`.

In [`fuseMag`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_fusion.cpp#L105),
`update_all_states = true` with `update_tilt = false` clears the first two
attitude-error gain entries. Its
[attitude injection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L736)
left-multiplies the nominal quaternion, so these error coordinates are in the
navigation frame. Clearing the horizontal entries retains only its vertical
rotation component.

The body-frame formula above is the corresponding coordinate transformation:

```text
Pi_n = e_D * e_D^T = diag(0, 0, 1)
Pi_b = R^T * Pi_n * R = u_b * u_b^T
```

The gain restriction is equivalent at a common linearization point with
consistent coordinate transforms. The projection formula is derived for our
right-error convention; it is not copied from PX4's gain-entry assignments.
The complete estimators are not equivalent: PX4 has earth-field and magnetic-bias
states and sequential component fusion, while ours uses a fixed reference field
and a batch correction.

PX4's [`mag_hdg` mode](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L185)
still uses magnetic-vector observations. It does not mean that only one scalar
yaw observation is fused, or that every non-heading state is frozen. The gain
restriction described here applies to `fuseMag`, not every PX4 update/reset path.

## Runtime field checks and gated fusion

The optional [`runtime::fuse_magnetometer`](../include/formal_eskf/runtime/magnetometer.hpp)
now applies this sequence to one AHRS/INS observation:

```text
field plausibility -> innovation gate -> existing magnetic correction
         |                   |                     |
     reject/error        reject/error         publish only on success
```

[`check_magnetic_field`](../include/formal_eskf/runtime/magnetic_field.hpp)
checks field strength and inclination against the supplied NED reference.
It rejects insufficient horizontal field in either vector. An additional wrapped
heading-direction check runs only with explicit caller-established independent
heading observability. It uses NED vector geometry, not Euler-angle extraction,
and does not initialize yaw or infer observability from the magnetometer itself.
The existing componentwise innovation gate always follows field acceptance,
even when the extra heading check is skipped.

Both stages expose their own diagnostics. Inspect `result.fusion.decision`
for the overall outcome and each stage's validity flag before reading metrics.
Only `fused` changes state/P. A numerical failure is distinct from measurement
rejection. Correction retains `ESKF_MAG_TILT`, full correlated V and INS bias
permissions; no core equations or defaults changed.

The behavior review used PX4 v1.16.2's
[`checkMagField` and `checkMagHeadingConsistency`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L455)
and [`fuseMag`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_fusion.cpp#L51).

| Concern | PX4 reference | This runtime boundary |
|---|---|---|
| Strength/inclination | Configurable checks; WMM or limited fallback; sustained-health timing | Explicit reference and thresholds, no fallback or timer; per-sample checks |
| Heading consistency | Filtered yaw discrepancy, alignment and horizontal-aiding/motion conditions | Instantaneous wrapped horizontal-direction discrepancy; caller explicitly supplies independent heading observability |
| Innovation | Any-axis outlier rejects the magnetic group before sequential fusion | Reuse the existing any-axis gate before our batch magnetic correction |

This is not a reproduction of PX4's controller or persistent health flags.
Thresholds use microtesla and radians (PX4 field units are Gauss); no values are
silently copied. A wrong attitude/reference can fail these checks, and some
disturbances can pass them. Passing does not establish physical sensor health.
Health persistence, fusion deadlines and source changes are now handled by the
optional lifecycle below. Startup alignment, actual estimator recovery, WMM
lookup and adapter wiring remain separate work. Requirements `R-MAGNETIC-FIELD` and
`R-MAGNETOMETER-FUSION` record the behavior; new Lean/ESBMC evidence is pending.

## Magnetic fusion lifecycle

[`runtime::MagnetometerFusion`](../include/formal_eskf/runtime/magnetometer_fusion.hpp)
owns only the temporal admission policy. The single-writer call order is:

```text
check_magnetic_field -> lifecycle.update (every tick, also without data)
                              |
                         fusion_allowed
                              |
                      fuse_magnetometer
                              |
                lifecycle.record_fusion(actual decision)
```

Feed the field check's successful, valid acceptance into `Sample::field_accepted`;
do not substitute innovation acceptance. The existing gated transaction rechecks
the same observation/prior before publication. Do not change the prior, reference,
calibration or check policy between qualification and that transaction. No-data
ticks use `Sample{}` and never call correction. This keeps the numerical APIs
independently callable, without adding a callback framework or scheduler.

| Event | Lifecycle action |
|---|---|
| Startup / short disturbance recovery | Require fresh, consecutive good samples spanning `health_time_us` before granting one sample's fusion permission |
| Bad field, stale data, disable or unready | Stop and clear health qualification; a short interruption may requalify without resetting the estimator |
| Innovation rejection | Consume the sample permission, retaining the last successful fusion time |
| Fusion deadline exceeded | Latch recovery; a good sample or enable toggle cannot bypass it |
| Numerical correction failure | Require recovery immediately; do not retry/reset the estimator internally |
| Device ID or calibration count changes | Clear old-source history, bind the new source and require explicit re-entry authorization |
| `restart()` | Caller has established safe re-entry; retain time/source records and wait for a new full good interval |

Configure three positive durations explicitly: `health_time_us`,
`maximum_sample_age_us` (also the maximum qualifying sample gap), and
`fusion_timeout_us`. There are no selected deployment defaults. Health uses
sample-time span with `>=`; freshness/progress expire at `>`. Restart clears
session progress but does not rewind timestamps or fabricate a successful fuse.
Reference/frame/check-policy changes and external yaw resets also require
caller-coordinated restart; full `reset()` is for an explicitly reinitialized
timeline/configuration. Neither changes state/P or emits platform reset notices.

### Lifecycle comparison with PX4

The pinned [magnetometer controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L46)
and [sensor/calibration change handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2481)
were reviewed at PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`:

- PX4 waits after field-check failures before starting and requires fresh samples
  and alignment conditions. Its filter warm-up, strict elapsed boundary and
  mode-specific continuation policy are not copied: this controller requires a
  complete new sampled-good interval after every health interruption.
- PX4 stops on missing data and handles fusion timeout by resetting magnetic
  states/possibly heading, or stopping, depending on aiding/motion. We retain
  deadline monitoring but require caller-directed recovery: our fixed-field
  INS/AHRS has no `mag_I`/`mag_B` states and cannot silently reuse those resets.
- PX4 detects device/calibration changes in the adapter and resets associated
  state/filter history. We detect the supplied identity/version changes and
  revoke permission; actual alignment/state/P repair remains outside this class.

This is a portable lifecycle with explicit recovery authority, not an EKF2
controller clone. `active` indicates sampled health qualification and eligibility,
not physical health or flight validity.
Tests in [`test_magnetometer_lifecycle.cpp`](../tests/test_magnetometer_lifecycle.cpp)
exercise transitions and actual AHRS/INS field-check/gate/correction sequences.
`R-MAGNETOMETER-LIFECYCLE` records the contract; Lean/ESBMC evidence is pending.

## Integration requirements and limits

- `ESKF_MAG_TILT=1` projects the attitude gain after the checked solve and bias
  permissions, before either state or covariance correction. The existing
  residual, Jacobian and full observation covariance are retained.
- The shared correction helper uses the same modified full gain `K'` in both
  updates, then performs the existing injection and covariance reset:

  ```text
  delta_x = K' * r
  A       = I - K' * H
  P_c     = A * P * A^T + K' * V * K'^T
  ```

  Projecting only `delta_theta_b` while leaving the original gain in the Joseph
  update would make the state and covariance updates inconsistent.
- The prior unit quaternion, symmetric PSD prior covariance and symmetric SPD
  observation noise remain caller premises. This operation does not normalize
  the prior or repair invalid covariance inputs. Existing finite, symmetry and
  solve checks are retained; they do not prove these mathematical premises.
- Non-finite projection arithmetic returns `non_finite_result`; other statuses
  propagate from the existing correction. Both outputs publish only on success and may independently
  alias their complete corresponding inputs. Failure preserves both outputs.
- Projection does not constrain other INS gain rows. Position, velocity and
  permitted biases can update; disabled bias rows remain zero. Bias changes can
  influence future tilt. Any further restriction is a separate policy decision.
- The runtime wrapper supplies instantaneous field checks and innovation gating;
  physical health, independent heading observability and recovery remain caller
  responsibilities. Tilt protection neither removes heading errors caused by
  interference nor guarantees useful heading information when the horizontal
  magnetic field is too small.
- A genuine scalar heading-observation model is a separate design involving
  tilt compensation, angular wrapping, its Jacobian and effective noise. It is
  not interchangeable with projecting a full-field gain.

## Verification status

Projection implementation validation on 2026-10-07 (before the runtime wrapper):

| Check | Result and scope |
|---|---|
| GCC Debug / Eigen CTest | 20/20 passed with default projection enabled. |
| Magnetometer configurations | Macro ON/OFF passed with GCC and Clang; projection ON also passed with PX4 matrix and first-order reset. |
| ASan / UBSan | 20/20 passed; LeakSanitizer disabled for this environment. |
| Cppcheck / clang-format | unix32 and unix64 analysis, and read-only formatting check passed. |
| Existing ESBMC regressions | 4/4 magnetic-wrapper profiles passed with explicit `ESKF_MAG_TILT=0`: AHRS/INS, binary32/binary64, separate outputs. Not a full proof-suite rerun or projection proof. |

CI retains its five existing build jobs: GCC checks projection ON with both
reset choices, and Clang checks OFF. No extra Cartesian-product jobs are added.

[`test_eskf_magnetometer.cpp`](../tests/test_eskf_magnetometer.cpp) covers AHRS/INS
and binary32/binary64, tilted and 90-degree-pitch attitudes, quaternion sign
equivalence, all 64 INS bias-permission combinations and all four independent
state/covariance alias arrangements. An independent long-double gain/Joseph
oracle checks the complete covariance; existing injection/reset is reused after
that oracle. A closed-form level case separately checks that heading updates
remain possible and protected tilt variance is not spuriously reduced.

Separate builds check projection enabled and disabled, projection overflow,
ordinary correction failures and the original full-field behavior when disabled.
Invalid macro values fail compilation, not a runtime status check. These are
regression tests, not formal proofs. The traceability entry `E-MAG-TILT` records the missing Lean/ESBMC evidence:
projection/tilt identities, consistent constrained gain, finite-precision failure
and publication behavior. Existing `E-OBS` and correction profiles cover the
unrestricted path only (`ESKF_MAG_TILT=0`); their results must not be extended to
the default protected build. The ESBMC runner explicitly selects `0` for E-OBS,
and the harness rejects a mismatched configuration instead of claiming projection
coverage.

Runtime-wrapper validation additionally covers field geometry and threshold
boundaries, angle wrapping, weak horizontal fields, tilted/90-degree-pitch
attitudes, invalid inputs/arithmetic, same-prior gate diagnostics, all INS bias
permissions, whole-object aliases and correction failure after a passed gate.
The tests are in [`test_magnetometer_fusion.cpp`](../tests/test_magnetometer_fusion.cpp).
GCC Debug/Eigen and ASan/UBSan each pass 21/21 tests (leak detection disabled);
targeted Clang/full-field, PX4 matrix and first-order-reset tests also pass.
Cppcheck unix32/unix64, read-only formatting and the 100 offline agent-review
regression tests pass. These are native/tooling checks, not a new Lean/ESBMC
proof or an AI semantic audit. The runtime's two new proof entries remain gaps.

Lifecycle validation adds timestamp/health boundaries, no-data ticks, pauses,
rejection deadlines, source/calibration changes and explicit re-entry. Tests also
drive the real field-check/gate/correction path for AHRS/INS and float/double.
GCC Debug/Eigen and ASan/UBSan each pass 22/22 tests (leak detection disabled);
the lifecycle target passes Clang Release with projection OFF and PX4 matrix
with projection ON. Cppcheck unix32/unix64, read-only formatting and the 100
offline agent-review regression tests pass. One invalid-enum regression assertion
has a local, explained Cppcheck `knownConditionTrueFalse` suppression; production
checks and analysis configuration are unchanged. No new Lean/ESBMC proof or
end-to-end PX4 adapter/flight validation is claimed.
