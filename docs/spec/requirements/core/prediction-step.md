# E-STEP-PRED: Combined prediction

**ID:** `E-STEP-PRED`

**Status:** Draft

**Applies to:** INS/AHRS predict transaction.

**Parent:** [E-STEP](composed-steps.md).

## Required behavior

`predict` calls nominal prediction first. Only if that succeeds, covariance prediction receives the old covariance and the same IMU, `dt` and parameters. INS covariance also receives the **old** nominal state; AHRS covariance does not consume nominal state. Publish both component results unchanged only after both succeed; otherwise return the first failure and preserve both outputs.

The shared [E-STEP](composed-steps.md) frame, alias, callee-evidence and configuration obligations apply.

## Acceptance and verification

Verify all stated call arguments/order, first-failure propagation and joint publication using matching component contracts; include every permitted whole-object alias combination and arbitrary initial outputs.
