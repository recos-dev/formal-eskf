# E-INNOV: componentwise innovation gate

**ID:** `E-INNOV`

**Applies to:** Shared core correction policy; configurations and limits specified below.

## Required behavior

This pure operation consumes a fixed-size innovation vector `r`, the diagonal of the innovation covariance `S = H P H^T + V`, and an explicit `gate_sigma > 0`. It does not compute S, validate its positive definiteness, apply correction or manage fusion lifecycle. Measurement-noise variances alone are not a substitute for `diag(S)`.

- Compute `test_ratio[i] = (r[i] * r[i]) / ((gate_sigma * gate_sigma) * S[i,i])`. Reject the whole observation if **any** computed ratio is greater than one; equality is accepted. This is not a joint Mahalanobis gate. No implicit minimum sigma, clamping or gate default is applied.
- All inputs must be finite; otherwise return `non_finite_input` before arithmetic. Nonpositive sigma or variance returns `domain_error`. Positive variances, the computed squared sigma and each denominator must be at least the scalar format's smallest positive normal value; smaller values return `zero_or_unsafe_divisor`. Isolated failure causes have these statuses; competing domain/safe-range violations have no universal precedence.
- Non-finite squared sigma, squared innovation, denominator or ratio returns `non_finite_result`. A large denominator must not overflow into a false acceptance. Small innovations or ratios may underflow; decisions use the computed IEEE-754 ratio, not an exact-real boundary guarantee.
- On success, publish every finite nonnegative ratio and the aggregate rejection flag, even when rejected. `success` reports valid diagnostics, not permission to fuse. The caller must require both success and no rejection before treating the gate as passed.
- On failure, preserve the entire diagnostic output. Its vector may alias either complete input vector; otherwise inputs are unchanged. Do not exit early on an outlier and miss invalid later axes. No estimator state or covariance is modified.

Lean establishes the exact-real ratio, sign invariance, inclusive boundary and componentwise aggregation. The `innovation` ESBMC suite checks the actual production body for binary32/binary64, dimensions 1/2/3 and all five whole-vector alias partitions, with arbitrary IEEE inputs and old diagnostics. It covers ordered statuses, every computed ratio and atomic publication; no arithmetic summary or success premise is used. Floating-point decisions need not agree with the exact-real boundary. Native tests provide additional regression evidence. `R-MEASUREMENT-FUSION` composes the gate with correction; sensor-specific settings and deployment wiring remain separate obligations. See the [verification scope and evidence rules](../../../guides/verification.md).
