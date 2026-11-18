# E-PCOV: Error covariance prediction

**ID:** `E-PCOV`

**Applies to:** Shared INS/AHRS core; binary32/binary64 under the selected build configuration.

**Uses:** [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-INPUT-DOMAIN](../common/input-domain.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Cheng and Huang, SICE 2023](https://doi.org/10.23919/SICE59929.2023.10354209), (14), (18), with the non-isotropic injection described after [Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (457). The project uses the explicit noise-source frame rather than the paper's isotropic shortcut.

## Required behavior

Use [E-STATE](state.md) ordering, the **prior** attitude R=R(q_nb), corrected body
force f=f_m-b_a and angular increment d=(omega_m-b_g)dt. The INS transition is

\[
F_x=\begin{bmatrix}
I&I\Delta t&0&0&0\\
0&I&-R[f]_\times\Delta t&-R\Delta t&0\\
0&0&A&0&-I\Delta t\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix},\quad
F_i=\begin{bmatrix}
0&0&0&0\\-R&0&0&0\\0&-I&0&0\\0&0&I&0\\0&0&0&I
\end{bmatrix}.
\]

For `ESKF_QUAT_APPROX=0` (default), A=R(Exp(d))^T; for `1`, A=I-[d]_x.
The latter is a first-order transition, not the exact derivative of normalized
Euler integration. The former makes the attitude block exact for the frozen
rate; it does not make the full transition an exact discretization.

Produce P_next=F_x P F_x^T + F_i Q_i F_i^T using [E-NOISE](process-noise.md).
In particular velocity noise is R Q_a R^T; identity injection is equivalent only
for isotropic Q_a. Retain full prior cross-covariances. Do not add direct position
noise or higher-order position couplings to this truncated model. AHRS uses
P_next=A P A^T+Q_w with externally bias-calibrated angular rate, and consumes no
absolute attitude, specific force, INS biases or gravity parameter.

The caller establishes symmetric PSD P. INS checks the prior quaternion residual
under [E-PRED](prediction.md); AHRS does not consume that quaternion or its
residual tolerance. Apply the prediction time/normalization-parameter checks and
finite checks to consumed inputs. INS position, velocity and gravity are not
consumed. Reject invalid source noise using E-NOISE. Finite-input overflow is
`non_finite_result`; propagate downstream checked-math failures. Check computed
covariance finiteness, then restore symmetry by averaging each off-diagonal
pair as 0.5*a + 0.5*b before publication. Failure preserves output; output may alias the complete prior P.
Input symmetry/PSD is not established by a runtime decomposition here.

## Acceptance and verification

Establish the selected real transition/noise relation and conditional PSD preservation, and check production arithmetic, validation, aliasing and failure frames in the declared profiles. Symmetry cleanup does not prove IEEE PSD or equivalence to a full continuous-time discretization.
