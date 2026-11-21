# SW-REPLAY: Traceable host replay aiding

**ID:** `SW-REPLAY`

**Status:** Draft

**Applies to:** Host PX4-log replay sensor-policy bindings and shared PX4/KITTI coordinate preparation within their declared input capabilities; no live PX4 timing equivalence is implied.

**Uses:** [SW-CORE](core-estimation.md), [R-MEASUREMENT-FUSION](../runtime/measurement-fusion.md). Each child declares its portable-policy dependencies.

## Basis

The host replay contracts below bind shared algorithms to recorded measurements,
available metadata and a replay clock. They support reproducible inspection of
sensor-policy behavior, not a replacement for the live module acceptance contract.

## Required behavior

Replay shall apply the declared quality, lifecycle, sampling and numerical policies
to recorded data using their specified event order. Unavailable clipping, rest,
health or reference data shall retain their child-defined meaning; replay shall
not fabricate metadata to make a correction execute.

Recorded outcomes shall distinguish permission, attempted correction, rejection,
successful correction and reset, retaining the child-defined timestamps and
diagnostics. A numerical failure shall follow the replay failure policy without
erasing earlier completed operations or reporting a successful run. Replay's
seed assumptions and same-horizon scheduling shall remain distinct from PX4
initialization and delayed/current-output behavior.

## Decomposition

Child applicability follows the selected replay options and available input
metadata. Legacy/KITTI fallbacks remain limited to the cases explicitly permitted
by the child contracts; missing metadata does not authorize silently disabling
checks required by the selected profile.

| Child requirement | Allocated obligation |
|---|---|
| [R-REPLAY-GEODESY](../integration/replay/geodesy.md) | Bind checked geodetic positions and vector bases to one fixed replay origin. |
| [R-REPLAY-GNSS-QUALITY](../integration/replay/gnss-quality.md) | Convert and qualify available receiver metadata. |
| [R-REPLAY-GNSS-LIFECYCLE](../integration/replay/gnss-lifecycle.md) | Bind ordered GNSS permissions, operations, feedback and reset records. |
| [R-REPLAY-GNSS-GATING](../integration/replay/gnss-gating.md) | Apply group gates and record vertical-recovery diagnostics. |
| [R-REPLAY-GRAVITY-CONDITIONS](../integration/replay/gravity-conditions.md) | Supply available rest, clipping, alignment and aiding conditions. |
| [R-REPLAY-GRAVITY-SAMPLING](../integration/replay/gravity-sampling.md) | Bind real IMU sampling, observation weight and recorded gravity outcomes. |
| [R-REPLAY-BIAS](../integration/replay/imu-bias.md) | Bind replay health, bias controls and process-noise selection. |
| [R-REPLAY-MAGNETIC-FIELD](../integration/replay/magnetic-field.md) | Bind field/reference checks and diagnostics. |
| [R-REPLAY-MAGNETIC-LIFECYCLE](../integration/replay/magnetic-lifecycle.md) | Bind magnetic permissions, recovery and outcome accounting. |
| [R-REPLAY-MAGNETIC-MODEL](../integration/replay/magnetic-model.md) | Bind geographic reference adoption and availability. |
| [R-REPLAY-MAGNETIC-OBSERVATION](../integration/replay/magnetic-observation.md) | Prepare and execute recorded magnetic observations. |

## Acceptance and verification

- Match actual input availability and event order to the child decisions and
  output records. Exercise successful aiding as well as unavailable/rejected
  paths; a log lacking required metadata cannot demonstrate a successful path.
- Check that operation counts follow actual operations: correction and reset may
  both occur for one event. Missing data, held predictions and rejected samples
  shall not increase successful-fusion counts.
- Evidence shall identify the input, configuration and source revision. Trajectory
  plots or agreement with a logged estimator are not independent accuracy truth,
  a complete module proof or hardware real-time acceptance.
