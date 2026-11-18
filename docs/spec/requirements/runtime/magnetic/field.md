# R-MAGNETIC-FIELD: instantaneous magnetic-field plausibility

**ID:** `R-MAGNETIC-FIELD`

**Status:** Draft

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

**Parent:** [SW-MAGNETIC](../../software/magnetic-aiding.md).

**Uses:** [E-STATE](../../core/state.md).

## Required behavior

`runtime::check_magnetic_field` consumes a unit `q_nb`, calibrated body-frame `m_b`, trustworthy NED observation reference `m_n`, an optional borrowed NED health reference `health_reference_n` (all microtesla), explicit `MagneticFieldParameters`, and a caller-supplied `heading_observable` condition. It owns no state, clocks, calibration or WMM lookup. This check is for already aligned magnetic aiding, not initial yaw alignment or proof of sensor health.

- Form `measured_n = R(q_nb) m_b`. For measured/reference NED fields compute strength, horizontal magnitude and Down-positive inclination `atan2(z, sqrt(x*x+y*y))`. Strength in the rotated frame equals body strength under the unit-quaternion premise; finite-precision decisions use the computed quantities.
- `MagneticFieldChecks` selects strength/inclination tests and whether an independent reference is required. A non-null `health_reference_n` supplies that reference for this call; null means unavailable. Passing `&m_n` explicitly assigns both roles to the observation field and remains a caller trust premise, not automatic WMM availability. Both physical checks are selected by default. With an available reference, compare selected absolute strength/inclination differences to configured tolerances. With no independent reference, a selected required-reference check rejects with `failures.reference`; otherwise selected strength uses the inclusive `[5,85]` microtesla range and inclination is skipped. Requiring a reference with neither test selected does not create a check. This health fallback never changes the observation vector `m_n`.
- The health-reference pointer is used only for selected strength/inclination tests when an independent reference is available. Validate its finite coefficients, nonzero vector and geometry before publishing diagnostics; unavailable or unselected health data is not consumed. The measured/observation vectors retain their existing validation and horizontal-field guard. A vertical health reference alone does not fail that guard. Heading checks always use `m_n`; no health reference is silently substituted into the observation. The pointer is never retained. The caller must keep a supplied reference alive for the call; no second availability flag or single-reference forwarding overload is used.
- Selection does not disable input/arithmetic validation, horizontal-field protection, the separately requested instantaneous heading check, filtered heading consistency, lifecycle or innovation gating. Consequently disabled physical checks do not authorize unsafe geometry or bypass correction checks.
- Require both horizontal magnitudes to be at least `minimum_horizontal_field`, regardless of heading-check selection or `ESKF_MAG_TILT`. This policy rejects fields with insufficient heading information; it is not a general vertical-field tilt-aiding API.
- Only when `heading_observable` is true and both horizontal fields are sufficient, compute `wrap_pi(atan2(measured_E, measured_N) - atan2(reference_E, reference_N))` and compare its absolute value with `heading_tolerance`. The caller must establish aligned heading with independent aiding and suitable motion/observability; neither magnetometer consistency nor a GNSS fix alone establishes that condition. False skips only this extra check, never the innovation gate. No heading filter, hysteresis or reset is implied.
- All consumed coefficients and all four parameters must be finite. Strength tolerance is nonnegative; both angle tolerances are in `[0, pi]`; minimum horizontal field is positive normal. Zero field vectors and invalid parameters return `domain_error`; non-finite inputs return `non_finite_input`. Default parameters are invalid until explicitly configured. No replacement observation field, tolerance clamp or implicit normalization is allowed.
- Non-finite rotation or squared-magnitude arithmetic returns `non_finite_result`; total squared magnitude below the smallest positive normal value returns `zero_or_unsafe_divisor`. Propagate checked square-root/atan2 failures. These checks intentionally reject extreme arithmetic domains rather than promise arbitrary-scale geometry. Inputs remain unchanged.
- Return fresh diagnostics only after all required arithmetic succeeds. On success, `diagnostics_valid` is true; named reference/strength/inclination/horizontal-field/heading failure flags determine `accepted`. Upper-threshold equality and minimum-horizontal equality pass. `strength_checked`, `inclination_checked` and `heading_checked` distinguish unperformed tests from passes. Unchecked inclination/heading errors are zero placeholders. `reference_strength` is the selected health-reference strength when available, the fallback center (45 microtesla) when unavailable, or zero when an available reference is unconsumed because both physical checks are disabled; it is not evidence that a WMM was supplied. Any failure returns `accepted=false` and invalid zero diagnostics. A current rejection is `success` with `accepted=false`, not a numerical failure.

Physical disturbance detection, attitude/reference correctness and threshold tuning
remain deployment assumptions. Native tests cover geometry, boundaries, frames,
all eight selections with/without a reference and invalid arithmetic in both
formats. ESBMC checks actual validation, checked scalar producers, ordered
strength/horizontal/inclination geometry and wrapped heading-error helpers.
Failure status, call arguments/order, finite geometry publication, output
atomicity and input purity are covered with the fixed-array proof backend.
The exact IEEE heading relation and failure frame use complementary successful
scalar and general failure profiles. Principal-range/finite guarantees separately
require atan2 results in the encoded [-pi,pi] range; finiteness alone is not that
contract. Lean proves corresponding exact real wrap/boundary properties.
The actual public plausibility caller is checked in both formats with null,
observation-identical, separate and body-aliased health references. Unconditional
status/call/failure frames compose with successful-computation diagnostics;
rejected fields remain included. All selection flags and numerical inputs are
symbolic. Same-type geometry/heading contracts bind every request and publication.
Input validation and health-data checks remain inline. Forward rotation is checked
against ordered R-times-body dots and its same-type matrix-coefficient producer;
real strength preservation reuses the unit-rotation norm theorem. These claims
cover reference reuse, unselected-data non-use, fallback limits, all thresholds,
complete diagnostics and borrowed-input purity. They do not establish accurate
math-library results, trustworthy sensor/reference data, observability or actual
adapter correspondence.

The solver-backed field/observation instantiation used by magnetic correction has
its own scalar-wrapper, geometry/heading frame, rotation, plausibility and
observation profiles. Its general scalar boundary admits zero root arguments
and arbitrary IEEE root/angle returns. The Cholesky positive-pivot observer stays
enabled in F-SOLVE; a raw-root equivalence check and factor regression establish
that changing this passive observer does not change values on the proved pivot
domain. This closes the backend boundary for field preparation, not the complete
public fusion transaction or library-accuracy claim. See the
[verification scope and evidence rules](../../../../guides/verification.md).
