# PX4-LIFE: initialization, failure and shutdown

**ID:** `PX4-LIFE`

**Status:** Draft

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

Startup validates the complete configuration and prepares storage, subscriptions
and publications. Alignment qualifies the required sensors, initializes the
filter and synchronizes the output predictor/history before valid output begins.
No diagnostic or controller may observe a partially initialized estimate.
Changing numerical, timing, capacity or source-selection configuration requires
stop/restart in this baseline; sensor metadata changes follow their source
contracts rather than silently reconfiguring the estimator.

Sensor rejection, source timeout and estimator failure are distinct:

- Rejection leaves that numerical operation's estimate unchanged and does not
  acknowledge fusion. Other permitted aiding may continue.
- Source timeout updates that source's lifecycle and the affected validity;
  it does not by itself reset the entire estimator.
- Numerical failure, unsupported IMU identity/calibration change or loss of
  required integration history latches an estimator fault. Stop further valid
  estimate publication, report the fault and require an explicit restart.

A callback is not a transaction across all sensors. Earlier successful updates
and an attitude already published before a later failure cannot be rolled back.
That attitude describes its recorded epoch and prior completed feedback; no
later publication may claim the failed fusion succeeded. Fault/age reporting
and downstream timestamp checks must make stale output unusable.

On shutdown, stop accepting callbacks, cancel scheduled work, synchronize with
in-flight work and diagnostic readers, then release resources. No callback or
reader may retain access after the estimator is destroyed.
