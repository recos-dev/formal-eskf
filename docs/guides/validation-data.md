# Validation data and replay claims

Replay commands live in the root README. Data selection and independent-oracle
acceptance belong to F-07.1; successful execution is not an accuracy result.

| Evidence | What it can support | Limit |
|---|---|---|
| PX4 ULog replay | Observed numerical behavior, source availability, fusion decisions and regression on the recorded inputs | An onboard EKF estimate is another estimate, not independent truth. Check topics, calibration, timing and license per log. |
| KITTI OXTS replay | Reproduction/comparison for the implemented replay pipeline | Do not label the OXTS navigation reference independent when related outputs are also used as measurements. State each signal's role. |
| PX4 SIH tests | Controlled simulated flight/fault scenarios against simulator state | A simulator can be wrong. Retain the known landing/contact failure; simulated truth does not qualify physical sensors or aircraft. |
| Future independent-reference datasets / hardware | Accuracy and recovery metrics within a defined sensor/calibration/envelope | Requires a compatible adapter, independent reference, declared error metrics and acceptance thresholds. No dataset is accepted merely because it is large or public. |

For every accepted run retain data identity/hash, source and tool versions,
profile, frames/units, timestamps/delays, initialization, sensor corrections,
oracle, accepted tolerances and failed/excluded intervals. Report loss/rejection
and validity as well as error statistics; do not silently remove failures.

The repository's replay programs are examples, not a general data-ingestion or
flight-qualification suite. Refer to `examples/replay_px4.py` and
`examples/replay_kitti.py` for supported inputs. A new dataset adapter or batch
campaign needs its own reviewed scope; there is no automatic thousands-of-logs
validation claim. Dataset availability and licensing must be checked at adoption.

See [F-07 validation work](../../FEATURE_TODO.md#f-07--deployment-validation),
[deployment scope](../../integrations/px4/README.md) and
[verification boundaries](../spec/verification/assurance.md).
