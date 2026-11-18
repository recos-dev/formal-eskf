# E-STEP-INJECT-RESET: Combined injection and reset

**ID:** `E-STEP-INJECT-RESET`

**Status:** Draft

**Applies to:** INS/AHRS inject_and_reset transaction.

**Parent:** [E-STEP](composed-steps.md).

**Uses:** [E-INJECT](injection.md), [E-RESET](covariance-reset.md).

## Required behavior

`inject_and_reset` injects the complete supplied error first. Only if that succeeds, covariance reset receives the original covariance and the error **before zeroing**. Publish the nominal result, reset covariance and zero in every error-mean coordinate only after both succeed. Either failure preserves all three outputs and returns that failure.

The shared [E-STEP](composed-steps.md) frame, alias, callee-evidence and configuration obligations apply.

## Acceptance and verification

Verify all stated call arguments/order, first-failure propagation and joint publication using matching component contracts; include every permitted whole-object alias combination and arbitrary initial outputs.
