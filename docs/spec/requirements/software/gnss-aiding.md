# SW-GNSS: Qualified GNSS aiding and recovery

**ID:** `SW-GNSS`

**Status:** Draft

**Applies to:** Portable GNSS policies and their caller composition; the R-GNSS-FUSION lifecycle profile uses GNSS as its sole horizontal and height source.

**Uses:** [E-STATE](../core/state.md), [R-MEASUREMENT-FUSION](../runtime/measurement-fusion.md), [E-OBS-POSITION](../core/position-observation.md), [E-OBS-VELOCITY](../core/velocity-observation.md).

## Basis

The child contracts separate receiver qualification, lifecycle permission and
numerical updates. This requirement states their composition obligations; it
does not prescribe one facade or allocate clocks and geographic projection to
the portable runtime.

## Required behavior

GNSS aiding shall distinguish received data, qualified data, authorized operations
and actual successful updates. A caller shall provide same-sample quality and the
required initialization/time context before acting on lifecycle permission, then
return the actual matching correction/reset outcome. Rejection, missing data and
failed numerical operations shall not be recorded as successful fusion.

Position, velocity and height groups shall retain the child-defined coupling,
independent deadlines and reset order. Bounded vertical-innovation recovery shall
not bypass receiver quality or lifecycle permission. Source/session changes shall
clear the histories required by the corresponding child contracts.

## Decomposition

| Child requirement | Allocated obligation |
|---|---|
| [R-GNSS-QUALITY](../runtime/gnss/quality.md) | Evaluate receiver checks and qualify continuous health. |
| [R-GNSS-FUSION](../runtime/gnss/lifecycle.md) | Grant single-use correction/reset permissions and account for actual progress. |
| [R-GNSS-RESET](../runtime/gnss/reset.md) | Apply checked position/velocity resets and covariance changes. |
| [R-GNSS-VERTICAL-RECOVERY](../runtime/gnss/vertical-recovery.md) | Apply the specified bounded vertical-innovation exception during IMU faults. |

## Acceptance and verification

- Establish quality-to-permission-to-operation-to-feedback correspondence for
  startup, ordinary aiding, rejection, timeout, re-entry and numerical failure.
  A controller proof cannot establish that its caller executed the operation.
- Check coupled horizontal progress and Down-velocity/height ordering under the
  child domains, with reset success separate from correction success.
- Integration claims shall identify their selected policies and evidence. The
  PX4 two-source [height manager](../integration/px4/height-fusion.md) has its own
  composition contract; it does not inherit the GNSS-only height policy merely
  because it reuses GNSS primitives. Receiver trust and physical accuracy remain
  caller/model assumptions.
