# R-ACCELEROMETER-FUSION: conditional gravity correction

**ID:** `R-ACCELEROMETER-FUSION`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [R-ACCELEROMETER-CONDITIONS](conditions.md), [E-OBS-ACCEL](../../core/gravity-observation.md).

## Required behavior

`runtime::fuse_accelerometer` composes `R-ACCELEROMETER-CONDITIONS` with the
unchanged `E-OBS-ACCEL` correction for AHRS and INS. No innovation gate is used.

- Supply the current body-frame specific force in m/s^2, not the LPF vector or
  delta-velocity. INS subtracts its prior bias exactly once inside the core;
  AHRS consumes externally bias-calibrated input. Preserve gravity magnitude,
  the existing Jacobian and caller-supplied effective V in m^2/s^4. Forward
  INS per-axis bias-update permissions unchanged; omission retains all enabled.
- The caller updates its magnitude filter once for each sample, independently
  of fusion enablement, and passes the actual result for that sample with
  conditions and prior state at the same fusion horizon. The caller owns
  timestamps, cadence and filter reset on source/calibration/frame changes.
  This stateless composition neither updates nor rolls back filter history.
- Evaluate conditions first. A failed magnitude result returns `failed` with
  its status. Unmet conditions return `rejected` with `Status::success` without
  calling correction or certifying its unused inputs. Otherwise call the
  checked correction: propagate failure status, and report `fused` only on
  success. Return the condition check separately; eligibility can remain true
  even if correction fails. No innovation diagnostics or test ratio is implied.
- Publish both state and covariance only on `fused`. Rejection or failure
  preserves both outputs. Each output may independently alias its respective
  complete prior; all other storage is disjoint. Separate inputs are unchanged.
  There is no retry, persistent mode, timer, source selection or reset policy.

Native tests cover real magnitude results, current-versus-filtered measurement
use, INS bias permissions, both configurations/scalar formats, all four alias
arrangements, rejection and numerical failures. ESBMC checks both actual public
fusion overloads and their inline condition predicate, binary32/binary64 and
all four independent whole-state/P alias modes. It composes the exact-type
`E-OBS` / `E-BIAS-UPDATE` correction frame: arbitrary success outputs and every
status, no writes on failure. Assertions bind every original request/output
argument, all six INS bias permissions, exact invocation count, result conditions,
status/decision, publication and all borrowed/disjoint input frames. No successful
or finite input premise removes rejected/failed paths. This control composition
needs no additional real-algebra theorem. Sample integration, magnitude-result
freshness and actual module/replay wiring remain separate obligations.
The composition does not establish a valid physical gravity observation,
noise independence, tuned V, full external-estimator equivalence or flight safety.

#### Replay sampling and tuning

See [gravity-sampling.md](../../integration/replay/gravity-sampling.md).
