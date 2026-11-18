# E-STEP-CORRECT: Complete correction transaction

**ID:** `E-STEP-CORRECT`

**Applies to:** INS/AHRS correct and magnetic correction transactions.

**Parent:** [E-STEP](composed-steps.md).

## Required behavior

`correct` must propagate solve, covariance-finalization and injection/reset failures without publishing either output. Its complete composition must use the component preconditions and result relations established by `E-CORRECT`, `F-SOLVE`, `E-INJECT` and `E-RESET`. Proving the two earlier combined APIs alone does not discharge this clause.

The shared [E-STEP](composed-steps.md) frame, alias, callee-evidence and configuration obligations apply.

## Acceptance and verification

Verify all stated call arguments/order, first-failure propagation and joint publication using matching component contracts; include every permitted whole-object alias combination and arbitrary initial outputs.
