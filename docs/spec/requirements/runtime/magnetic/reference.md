# R-MAGNETIC-REFERENCE: geographic magnetic reference adoption

**ID:** `R-MAGNETIC-REFERENCE`

**Status:** Draft

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

## Required behavior

`runtime::MagneticReference` consumes an externally supplied declination D,
inclination I (radians), and strength B (microtesla) in the estimator's NED frame.
It owns the adopted reference, revision and check timing, not geographic lookup,
position validity, calibration, estimator state/P or learned magnetic states.

- Form `m_n = B * [cos(I) cos(D), cos(I) sin(D), sin(I)]`. D is east-positive;
  I is Down-positive. Require finite values, D in `[-pi,pi]`, I in
  `[-pi/2,pi/2]`, and positive normal B. Propagate checked scalar failures and
  reject non-finite generated coefficients. Weak horizontal field remains the
  downstream field/heading guard's responsibility.
- Call at strictly increasing nonzero fusion timestamps. `origin_updated`
  requests an immediate check; otherwise a caller-valid position permits the
  first check, then checks strictly **more than 10 seconds** apart. An origin
  event is a lookup request, not authorization to change the navigation frame.
  A skipped check does not consume/validate model values or alter the reference.
- Adopt the first valid model, or a change strictly exceeding **1 degree** in D
  or I, or **1 microtesla** in B, relative to the last adopted model. Equality
  retains the old model. Compare angles directly, including across the pi cut,
  following the reference implementation. Small changes accumulate against the
  adopted model, not the last observed value.
- Every adoption advances `reference_id` from zero (uninitialized) to one, then
  monotonically; overflow fails instead of wrapping. Unchanged successful checks
  advance only the check clock. Failed calls preserve the model, revision and
  both timing watermarks, allowing a corrected retry at that timestamp. Return
  separate `checked` and `changed` diagnostics; a failed check never publishes a
  new reference or authorizes a reset.

The schedule and change thresholds follow
[PX4 v1.16.2 magnetic control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp).
This API additionally reports invalid domain/time and scalar failures explicitly,
and preserves history on failure. Native tests cover
binary32/binary64, two backends, axes/units, timing, thresholds and failure
preservation. Lean proves the real NED conversion's squared strength, horizontal
magnitude, north-axis and Down-component identities. It also proves both NED
basis orthogonality orders, positive determinant, and local-to-origin frame
identity, inverse, composition and squared-strength preservation. The basis
model takes radians; degree conversion, IEEE execution, geographic provenance
and raw platform libm in `MagneticModel::to_origin_frame` remain separate.
ESBMC checks actual
initialization/reset, validation, checked trigonometry, adoption and update in
both scalar formats with the fixed-array proof backend. The scalar boundary
admits every IEEE sine/cosine return. All failure frames/statuses are checked;
the successful-wrapper path separately proves each ordered IEEE coefficient.
Revision overflow, strict timing/change boundaries, skipped checks, caller-input
purity and the complete owner frame are covered. Clock order, finite field and
adopted-model domain have an initialization base and inductive public update
step. No successful-public-input assumption removes invalid calls.
Target libm accuracy, external model accuracy/version and actual adapter
lookup/origin correspondence remain separate obligations.
