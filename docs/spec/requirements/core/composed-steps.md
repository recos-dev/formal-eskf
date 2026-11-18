# E-STEP: Atomic composed operations

**ID:** `E-STEP`

**Applies to:** Sequential shared-core operations on valid objects; no concurrency or exception recovery.

## Required behavior

These clauses refine `E-STEP`; they are not additional requirement families. They concern sequential calls on valid C++ objects, not concurrency, exception recovery or a new predict-plus-correct API. Numerical validity remains owned by the component and numerical requirements.

- [E-STEP-PRED](prediction-step.md) owns this transaction.
- [E-STEP-INJECT-RESET](injection-reset-step.md) owns this transaction.
- **E-STEP-FRAME** — Each output may independently alias its respective complete input object; all other input/output storage is disjoint. Cover all four prediction and eight injection/reset arrangements. Distinct inputs, unused output objects, IMU and parameters are unchanged. Do not publish caller outputs before the last required callee succeeds. Initial outputs and scalar fields are arbitrary, including non-finite values. Preservation concerns values, signed zero and NaN classification, not padding or NaN payloads.
- [E-STEP-CORRECT](correction-step.md) owns this transaction.

Compositional caller proofs may abstract a callee's result and status, but must check its arguments and call order and identify matching frame evidence. Use scratch for results followed by another fallible operation; a summary may overwrite that scratch even on failure. The final fallible callee may receive the caller's output only when its unchanged-on-failure and alias contracts are independently proved for the matching instantiation. After its success, only infallible publication may remain. Transaction rollback must not depend on successful or unchanged scratch data. A callee declaration or a proof using an incompatible backend instantiation is not sufficient evidence. INS/AHRS, binary32/binary64 and each applicable prediction/reset mode must be accounted for.
