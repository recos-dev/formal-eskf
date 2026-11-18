# R-PX4-MAGNETIC-MODEL: PX4 module geographic magnetic reference

**ID:** `R-PX4-MAGNETIC-MODEL`

**Status:** Draft

**Applies to:** PX4 module magnetic caller binding; portable policies retain their own contracts.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-MAGNETIC-REFERENCE](../../runtime/magnetic/reference.md).

## Required behavior

The configured host-lookup interval and the portable reference check cadence are
separate. `R-MAGNETIC-REFERENCE` retains its strictly-greater-than-10-second
periodic check unless an origin event requests an immediate check. A shorter host
lookup interval does not force adoption: consume the returned `checked`/`changed`
result, not lookup success alone.

`MagneticModel` connects `R-MAGNETIC-REFERENCE` to the module without owning
lookup I/O, estimator state/P or magnetic bias states. The host's WMM table and
epoch remain external dependencies. No table is copied into formal-eskf.

- Query the host at the qualified GNSS origin during stationary startup. Later,
  on fresh usable magnetic samples, query strictly more than the configured lookup interval (default 10 s) after
  the last attempt while the module's own horizontal position is valid. Reproject the
  **estimated position**, not an unchecked incoming GNSS fix. An origin change
  requests an immediate lookup, using the origin if position is invalid. This
  does not add an origin-rebasing API or change the estimator's coordinates.
- Host D/I are degrees and strength is gauss. Convert to radians/microtesla,
  rotate local geographic NED through ECEF into the fixed origin NED, then feed
  D/I/B to the existing adoption policy. The >1 degree / >1 microtesla thresholds
  apply in this common frame. This frame handling differs from directly using
  local geographic NED; it is required by our fixed-frame estimator convention.
- A missing/invalid lookup cannot replace either reference or authorize a reset.
  Retain an already accepted model and record the unavailable attempt. Attempts,
  including failures, are spaced by the same configured interval rule. Without any accepted
  model, use the existing broad 5–85 microtesla strength fallback, with model
  availability false; a measured seed is not WMM evidence.
- Read `FESKF_DECL_TYPE` (mask 0..3, default 3) and `FESKF_MAG_DECL`
  (east-positive degrees in [-180,180], default 0) at module start. Invalid settings
  reject startup; changes require module restart. Bit 0 selects available geographic
  heading; otherwise bit 1 permits a finite nonzero stored declination; otherwise
  select zero/magnetic north. WMM health checking remains independent of both bits.
  This follows PX4's executable `getMagDeclination()` fallback order, without its
  learned-earth-field branch or automatic parameter saving.
- Geographic mode uses the adopted field for observation, heading consistency
  and reset. In saved/zero mode those operations use the stationary seed rotated
  by the selected yaw; only health checks use the WMM field. Both references share
  the same NED frame. Initialization waits for the existing 5 microtesla minimum
  horizontal seed field before assigning heading.
- Only a geographic-heading adoption advances the observation revision supplied
  to the lifecycle. A health-only adoption advances the model revision, leaves
  the observation field/revision exactly unchanged and cannot itself request a
  yaw reset. If an initially unavailable geographic source later becomes available,
  adoption follows the same real lifecycle/reset permission rules, not an implicit
  quaternion assignment. Reference changes retain the measurement LPF history.

ESBMC checks the actual `MagneticModel` construction, initialization, lookup
admission, update and public reference views with the fixed-array proof backend
in both scalar formats. The independently checked `MagneticReference` transition
closes its state contract. Initialization is atomic, including seeds borrowing
the receiver's prior observation/health fields; successful commit consumes the
old seed before changing that owned field. Update attempts advance the
lookup clock even on absent/failed models, while health and observation revisions
remain distinct. Construction and both mutators establish/preserve bounded lookup
counts, clock order and source/declination domains. Raw seed hypot/sin/cos returns
remain arbitrary pure platform boundaries: no finite-observation or libm accuracy
claim follows from this control proof. C++ origin-frame conversion and host lookup
I/O/sequencing remain separate.

Native tests exercise source selection, timing, frame conversion, invalid/missing
model retention and real correction/reset integration. SITL checks the selected
source and advancing periodic lookup count. They complement the scoped proofs;
none establishes calibration, model-epoch accuracy or long-distance flight qualification.
