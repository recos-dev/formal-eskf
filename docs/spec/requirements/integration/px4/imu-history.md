# R-PX4-IMU-HISTORY: delayed IMU timeline

**ID:** `R-PX4-IMU-HISTORY`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

`px4::ImuHistory` owns the retained intervals and the completed prediction cursor
under [the deployment profile](../../../profiles/px4-sih-250hz.md).
It does not own the numerical state, output predictor, observations or a clock.

- Validate signed startup settings before conversion. Accept only the implemented
  250 Hz mode and a lag covering source arrival/delay bounds within the fixed
  capacity. Derive the active length; invalid configuration preserves the old
  configuration. Reconfiguration after seeding or fault requires explicit reset.
- Seed from one validated input endpoint and its preceding measured rate; seed
  history remains empty. Reject invalid/repeated seeds without changing state.
- Admit ordered nonzero endpoints, separate valid integration periods, finite
  mean rates and permitted clipping. Check arrival age, source/calibration,
  interval spacing and overlap tolerance before writing history.
- Store the complete real input and a separate preceding-rate hold when the
  permitted gap exceeds tolerance. Preserve the measured periods; a synthetic
  hold is not new sensor evidence. Record the previous endpoint as interval start.
- Wait for the configured number of real intervals and valid actual lag. Never
  overwrite an unprocessed interval or skip it to repair excessive lag. Check
  the prospective retained span before mutation; an admission fault preserves
  history and cursors, latches its cause and blocks further use until reset.
- Selection is non-consuming. Advance the fusion cursor only on completion of
  the oldest eligible interval whose start equals the previous cursor. Wrong,
  premature or repeated completion faults without advancing. Numerical success
  is a caller premise; the timestamp acknowledgment cannot prove it occurred.
- Expose the latest admitted real interval without consuming it, including
  during warmup; return no interval before admission or after a latched fault.
- Reset clears stored inputs, cursors and fault, retaining configuration. All
  storage/work is bounded, with no dynamic allocation or platform effects.

The current evidence is native binary64 testing on Eigen and PX4 matrix, including
default/full active lengths, wrap, faults and timestamp limits. Formal source
proof remains open. `R-RING-BUFFER` does not extend its proof to this owner's
payload/capacity, time arithmetic, acknowledgment protocol, adapter or WCET.
