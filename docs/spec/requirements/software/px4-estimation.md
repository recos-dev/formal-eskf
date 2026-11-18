# SW-PX4: Time-consistent PX4 estimator integration

**ID:** `SW-PX4`

**Status:** Draft

**Applies to:** The single-owner PX4 INS module under the declared [SIH 250 Hz profile](../../profiles/px4-sih-250hz.md); other targets require their own acceptance evidence.

**Uses:** [SW-CORE](core-estimation.md), [C-CPP-SUBSET](../common/cpp-subset.md), [C-LAYER-BOUNDARY](../common/layer-boundaries.md), [R-RING-BUFFER](../runtime/ring-buffer.md). Sensor bindings below identify the portable policies they consume.

## Basis

The [PX4 architecture](../../architecture/px4-module.md) allocates the existing
module contracts. This parent defines the complete integration obligation;
child sources retain their pinned upstream references and project adaptations.

## Required behavior

The module shall maintain a delayed state/covariance estimate and a current-time
output using admitted IMU history and eligible, time-aligned observations. It shall
provide the configured sensor aiding and output under one frozen session
configuration, one estimator owner and bounded callback work. Feedback shall
match delayed main and output epochs; published state, validity, covariance proxies
and reset information shall retain their declared horizons and reference points.

Initialization, source loss, rejected observations and estimator failure shall
follow distinct lifecycle rules. Missing history or failed numerical operations
shall not be disguised by advancing timestamps or acknowledging successful fusion.
Publication shall follow PX4-OUT and PX4-LIFE: a later failure cannot retract an
earlier current-attitude publication, and the whole callback is not an atomic
transaction. Timing/resource acceptance is required for the claimed deployment.

## Decomposition

Feature applicability follows PX4-CONFIG and the respective sensor binding.
A disabled feature retains its specified fallback and diagnostic obligations;
it is not grounds to omit the entire child from review. Build exclusion and
startup disablement may have different effects, as specified for gravity aiding.

| Child requirement | Allocated obligation |
|---|---|
| [PX4-OWN](../integration/px4/ownership.md) | Own estimator state, inputs and diagnostics across execution contexts. |
| [PX4-CONFIG](../integration/px4/configuration.md) | Validate and freeze the selected deployment configuration. |
| [PX4-LIFE](../integration/px4/lifecycle.md) | Initialize, handle failures and stop with safe resource lifetimes. |
| [PX4-EXEC](../integration/px4/execution.md) | Order bounded callback admission, propagation, aiding and feedback. |
| [PX4-TIME](../integration/px4/timestamps.md) | Define time domains, retention, source admission and loss policy. |
| [R-PX4-IMU-HISTORY](../integration/px4/imu-history.md) | Retain and advance delayed IMU intervals without skipping required history. |
| [R-PX4-OBSERVATION-HISTORY](../integration/px4/observation-history.md) | Select eligible observations with historical context and source generation. |
| [R-PX4-DELAYED-FUSION](../integration/px4/delayed-fusion.md) | Connect the histories, delayed updates and output feedback in the live module. |
| [R-PX4-GNSS-QUALITY](../integration/px4/gnss-quality.md) | Bind receiver metadata and context to GNSS qualification. |
| [R-HEIGHT-FUSION](../integration/px4/height-fusion.md) | Coordinate GNSS/barometer height, offsets and recovery. |
| [R-PX4-BARO-GROUND-EFFECT](../integration/px4/barometer-ground-effect.md) | Apply configured ground-effect residual handling. |
| [R-PX4-IMU-BIAS](../integration/px4/imu-bias.md) | Bind IMU health, process noise, bias permissions and value limits. |
| [R-PX4-GRAVITY](../integration/px4/gravity-aiding.md) | Bind interval gravity observations to delayed-horizon permissions. |
| [R-PX4-MAGNETIC-MODEL](../integration/px4/magnetic-model.md) | Supply geographic magnetic-reference updates. |
| [R-PX4-MAGNETIC-AIDING](../integration/px4/magnetic-aiding.md) | Bind magnetic qualification, correction and heading recovery. |
| [R-PX4-OUTPUT-PREDICTOR](../integration/px4/output-predictor.md) | Propagate current output and apply matching delayed feedback/resets. |
| [R-PX4-SENSOR-POSITIONS](../integration/px4/sensor-positions.md) | Convert sensor observations and output between physical reference points. |
| [R-PX4-FRAME-OUTPUT](../integration/px4/output-frames.md) | Convert geographic and attitude-variance output frames. |
| [R-PX4-STATUS](../integration/px4/status.md) | Report confidence, progress, uncertainty and failures with their actual meaning. |
| [PX4-OUT](../integration/px4/publication.md) | Publish coherent estimates, sample epochs and reset notifications. |
| [PX4-RT](../integration/px4/real-time.md) | Bound allocation, synchronization, memory and execution for the target profile. |
| [PX4-VERIFY](../integration/px4/acceptance.md) | Define module acceptance evidence and change-impact handling. |

## Acceptance and verification

- Trace an admitted interval and observation through warmup, selection, prediction,
  aiding, feedback and publication using production module paths. Establish all
  applicable child obligations, including their interconnections.
- Exercise delay/jitter, duplicate/lost inputs, generation changes, resets,
  no-IMU expiry, numerical/publication failure and shutdown. Verify both horizons
  and the state/validity/reset information actually visible to consumers.
- Apply PX4-VERIFY and PX4-RT for the selected profile. Component proofs and
  passing host/SIH scenarios do not establish target deadlines, physical flight
  safety, all PX4 features or an error bound for current-time output tracking.
