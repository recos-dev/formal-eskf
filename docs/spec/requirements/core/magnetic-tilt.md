# E-MAG-TILT: magnetic tilt protection

**ID:** `E-MAG-TILT`

**Status:** Draft

**Applies to:** Shared core correction policy; configurations and limits specified below.

**Parent:** [SW-CORE](../software/core-estimation.md).

## Required behavior

AHRS/INS magnetic correction uses compile-time `ESKF_MAG_TILT=1` by default; `0` selects the original unrestricted full-field update. Both use the same full-vector residual, Jacobian and observation covariance; protection is not a scalar heading-observation model. No runtime mode argument or selection branch is permitted. The setting must be consistent across translation units, with values other than 0/1 rejected at compile time. See [the geometric explanation and API](../../../guides/magnetic-tilt-protection.md).

- Under the existing unit-prior premise, form `u_b = R(q_nb)^T * [0,0,1]^T` from the prior attitude. After the gain solve, replace only its attitude block by `K_theta' = (u_b * u_b^T) * K_theta`. Use AHRS rows 0..2 and INS rows 6..8. Do not select body X/Y axes or change H.
- Preserve other INS gain rows, except for disabled bias rows under `E-BIAS-UPDATE`. Use this same final gain in the error correction and full Joseph update, followed by existing right-multiplicative injection and covariance reset.
- In exact arithmetic, the injected rotation preserves `R(q_nb)^T * [0,0,1]^T`; only the navigation-vertical rotation component changes. A floating-point tilt-error bound is a separate obligation. Other-state updates, later prediction and magnetic observability are not constrained by this identity.
- When protection is enabled, check the computed vertical axis, projector and projected gain for non-finite results, returning `non_finite_result`. Retain existing input/covariance checks and downstream status propagation. Unit prior and PSD/SPD remain caller premises; no implicit repair is introduced.
- Publish state and covariance together only on success. Preserve both on any failure and support all four independent whole-state/covariance alias arrangements. `ESKF_MAG_TILT=0` must agree with the original full-field behavior and must not compile the projection path.

Lean establishes the exact unit-axis projector identities and tilt preservation under right-multiplicative injection, including exact normalization of any nonzero parallel-axis increment candidate. This includes the Taylor candidate without claiming its angle equals exact Exp. The `magnetic` ESBMC suite checks actual observation preparation, ordered projector/gain products, M1 AHRS/INS transactions, all four state/covariance aliases and symbolic INS bias masks in binary32/binary64. It composes with the existing same-backend rotation, correction/Joseph and injection/reset producers. E-OBS and the `bias` suite retain M0 coverage; native tests in both modes supplement these source claims. Floating-point tilt-error bounds, external backends and target execution remain separate obligations. See the [verification scope and evidence rules](../../../guides/verification.md).
