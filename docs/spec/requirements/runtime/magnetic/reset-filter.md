# R-MAGNETOMETER-FILTER: magnetic data for heading reset

**ID:** `R-MAGNETOMETER-FILTER`

**Status:** Draft

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

**Parent:** [SW-MAGNETIC](../../software/magnetic-aiding.md).

## Required behavior

`runtime::MagnetometerFilter<Linalg>` owns a three-vector body-field LPF for reset
only. First sample or explicit reseed stores the finite input exactly; otherwise
`filtered += alpha * (sample - filtered)`, with fixed `alpha = 0.01/(0.09+0.01)`.
Call once per consumed magnetic sample, not per elapsed-time interval. This follows
PX4's nominal 10 ms / 90 ms filter setup; replay sample selection/cadence remains
its own and does not establish bitwise equivalence or a calibrated cutoff.

- Input is already calibrated body-FRD field in microtesla. No extra sensor bias
  or earth-field estimate is introduced. Reference-only changes preserve history.
- Non-finite input returns `non_finite_input`; non-finite computed candidate returns
  `non_finite_result`. Failure preserves history/initialization, including a reseed.
  `reset()` discards history. The caller must abort or restart lifecycle/filter
  coordination after a filter failure; it must not continue a changed-source
  session with old-source filter history.
- This LPF does not authorize correction, prove field health or change normal
  correction input. Reset uses the filtered vector and the checked heading-reset
  operation; source/time/readiness/field qualification belongs to the lifecycle.
  Native float/double tests cover seeding, filtering, reseeding and failure
  atomicity. ESBMC checks actual construction/reset and every update branch in
  both formats with the fixed-array proof backend: reseed and first-sample
  precedence, exact scalar-format recurrence, failure priority, complete history
  frame, input purity and accessor identity. Finite history has a base and an
  inductive update step, not a finite trace argument. The existing real affine
  filter, constant-input, convex-bound and residual theorems apply component-wise
  with this same fixed coefficient; they are reused, not counted as new theorems.
  Adapter cadence/source coordination and physical filtering-error bounds remain
  separate obligations.
