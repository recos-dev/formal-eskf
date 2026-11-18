# PX4-OUT: output prediction and publication

**ID:** `PX4-OUT`

**Status:** Draft

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

Output prediction uses full-rate admitted IMU data and the latest accepted
bias/feedback. Compare output history and the main estimate at the same delayed
epoch to construct tracking corrections; apply them to the current output
without pretending that delayed state was measured at the current time.
Use the PX4 output-observer structure with our quaternion/frame conventions and
numerical contracts. Track attitude, velocity and position tracking error.

Successful filter resets must update affected output history/current state and
publication bookkeeping coherently before the next affected output. Reset
counters advance once per actual reset; reported deltas describe the output
change seen by consumers. Ordinary tracking corrections are not reset events.

The main filter and predictor describe the IMU point. The module converts GNSS
antenna observations to that point and current navigation output to the configured
body origin under [R-PX4-SENSOR-POSITIONS](sensor-positions.md#r-px4-sensor-positions-sensor-and-output-reference-points).
Keep installation calibration frozen for the session; include orientation-induced
body-point changes in reset notifications. This nominal transform does not add
installation uncertainty or transport covariance to the output point.

Publish sample epoch separately from publication time. Document the horizon of
each covariance/uncertainty field; delayed covariance must not be presented as
a newly propagated current-time covariance. Alignment, active aiding, freshness,
dead reckoning and numerical health must remain separate validity inputs.
Topic mappings retain the existing sensor/reset requirements and PX4 meanings.
Rotation-variance frames and computed longitude follow
[R-PX4-FRAME-OUTPUT](output-frames.md#r-px4-frame-output-geographic-and-rotation-variance-output-frames).
Capability, uncertainty, bias, innovation and fault reporting follow
[R-PX4-STATUS](status.md#r-px4-status-confidence-and-failure-reporting).
Its covariance proxies and snapshot counters are not calibrated accuracy bounds
or lossless event records.
Advertisement/publication failure must be observable and cannot count as
successful delivery. Ordinary absence of subscribers is not a delivery failure.
