# E-CORRECT: Measurement correction and Joseph covariance

**ID:** `E-CORRECT`

**Status:** Draft

**Applies to:** Shared INS/AHRS core; binary32/binary64 under the selected build configuration.

**Parent:** [SW-CORE](../software/core-estimation.md).

**Uses:** [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-INPUT-DOMAIN](../common/input-domain.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (274)-(275) and Joseph form in footnote 26.

## Required behavior

The caller supplies r=z-h(x), H=dh/d(delta_x) at the same prior, prior error mean
zero, symmetric PSD P and symmetric SPD V. Measurement noise is independent of
prior error in this model. Measurement dimension is compile-time and independent
of nominal quaternion storage. Keep the full V, including off-diagonal terms.

\[
C=PH^T,\quad S=HC+V,\quad KS=C,\quad
\widehat{\delta x}=K_{eff}r,\quad
P_c=(I-K_{eff}H)P(I-K_{eff}H)^T+K_{eff}VK_{eff}^T.
\]

Use [F-SOLVE](../math/linalg/spd-solve.md) for the right SPD solve; do not explicitly
invert S. K_eff is K with any [bias permissions](bias-update.md) and, for the
magnetic wrapper, [tilt protection](magnetic-tilt.md) applied. The same final gain
and **same prior P** shall enter both error and Joseph calculations. Generic
correction does not select a sensor model, gate or wrap a residual.

Check finite P,r,H,V and normalization threshold first. Require threshold in
(0,1], exact input symmetry, nonnegative P diagonal and positive V diagonal;
violations are `domain_error`. These necessary checks do not establish full
PSD/PD. Preserve caller matrices rather than silently symmetrizing inputs.
Non-finite C, S, error, I-K_eff H or covariance arithmetic is
`non_finite_result`; propagate solve/injection/reset failures. Computed S receives finite checks and symmetry cleanup. P_c receives finite
checks and either dense symmetry cleanup or generated upper-triangle mirroring. Success of this arithmetic is not
an IEEE PSD guarantee.

`correction_covariances` shall return C and S together, without solving or reading
the nominal state. It applies the same matrix/r/threshold validation; its two
outputs are disjoint from one another and the inputs. A successful result is
valid only for that prior/linearization and is not permission to skip later
correction validation.

`correct` shall inject the estimated error and reset P_c under
[E-STEP-CORRECT](correction-step.md). Its returned covariance is the **post-reset**
covariance. No caller output may change until the entire operation succeeds.
Nominal validity is also subject to injection's checks; the unit-prior premise
is not established by matrix validation.

`ESKF_CODEGEN=1` (default) selects generated INS matrix arithmetic for the
[built-in kernel scope](../../../../codegen/README.md#generation-scope); `0`
selects dense products. AHRS remains dense. Preserve the stated input premises,
checked producer/intermediate boundaries and atomic publication in either mode.
Generated covariance kernels mirror their upper triangle and receive finite
checks without a second pairwise average; dense products retain symmetry cleanup.
Algebraic equivalence under the premises does not require identical rounding,
overflow or boundary decisions for all IEEE inputs.
Generic arbitrary-H correction products remain dense. Known INS observations
select their model explicitly at compile time in both gate preparation and
correction. Measurement dimension alone shall not select a model; full V and the
same constrained K_eff remain inputs to Joseph. INS post-injection reset follows
the configured arithmetic mode, including after generic correction.

## Acceptance and verification

Establish exact gain/Joseph relations and conditional symmetry/PSD, then verify matrix validation, solve dependency, constrained-gain consistency and the complete transaction. Numerical conditioning, backend accuracy and model/noise validity remain separate obligations.
