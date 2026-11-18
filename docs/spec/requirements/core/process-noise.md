# E-NOISE: Discrete process-noise covariance

**ID:** `E-NOISE`

**Status:** Draft

**Applies to:** Shared INS/AHRS core; binary32/binary64 under the selected build configuration.

**Uses:** [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-INPUT-DOMAIN](../common/input-domain.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Cheng and Huang, SICE 2023](https://doi.org/10.23919/SICE59929.2023.10354209), (15), (18), and [Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (457). Per-axis variances extend the isotropic paper model.

## Required behavior

Let D_a and D_w contain the three nonnegative body-axis per-sample specific-force
and angular-rate variances. Let D_ba and D_bg contain bias random-walk variance
**densities**. These are diagonal matrices; independent sources and axes are
assumed. The INS operation shall produce the 12x12 source-coordinate covariance

\[
Q_i=\operatorname{diag}(D_a\Delta t^2,D_w\Delta t^2,
                         D_{ba}\Delta t,D_{bg}\Delta t).
\]

The AHRS operation shall produce only the 3x3 angular block D_w dt^2. It shall
not inspect unused INS noise fields. Sample variances are not continuous noise
densities: the caller shall convert sensor specifications to this convention.
Units of the four resulting blocks are m^2/s^2, rad^2, m^2/s^4 and rad^2/s^2.
Zero variances are allowed. Off-diagonal coefficients shall be zero.

Check finite dt, then dt > 0; failures are `non_finite_input` and `out_of_range`,
respectively. This operation does not apply prediction's dt_min/dt_max interval.
For each consumed source group in the order above, reject any non-finite variance
before a negative variance (`non_finite_input`, then `domain_error`). Evaluate
sample scaling as `(variance * dt) * dt` and density scaling as `density * dt`;
reject non-finite generated coefficients as `non_finite_result`. Publish the
complete matrix only on success; failure preserves the output.

This operation supplies source noise. [E-PCOV](covariance-prediction.md) owns
its mapping into error coordinates, including anisotropic body-to-navigation
rotation. It shall not be applied twice.

## Acceptance and verification

Verify the diagonal/time-scaling relations and nonnegativity in exact arithmetic; verify checked execution, source selection, ordered scaling and failure preservation for each scalar format. These checks do not establish sensor-noise calibration or long-run filter consistency.
