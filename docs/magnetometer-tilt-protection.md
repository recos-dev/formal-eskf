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
- Magnetic-field health, heading observability, innovation gating and recovery
  remain necessary. Tilt protection neither removes heading errors caused by
  interference nor guarantees useful heading information when the horizontal
  magnetic field is too small.
- A genuine scalar heading-observation model is a separate design involving
  tilt compensation, angular wrapping, its Jacobian and effective noise. It is
  not interchangeable with projecting a full-field gain.

## Verification status

Validation on 2026-10-07:

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
