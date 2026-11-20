# Test Implementation Actions

**Status:** Draft

Implements [V-02](../../../roadmap.md#v-02--design-and-execution-evidence-pilot)
and the [core/runtime design](README.md). The project owner authorized execution
and per-file commits; requirement and numerical-criterion approval remain separate.
Integration owners and production behavior changes remain outside this work.

## Execution rules

- Each row has one primary implementation file and one reviewable commit. Essential
  build registration, source moves and traceability/case-binding edits accompany
  that file; unrelated implementation changes do not. Shared helpers are separate
  rows. A necessary production refactor gets its own per-file action and validation.
- Read the canonical requirement, planned partitions and current assertions. Reuse
  meaningful tests, add missing checks and remove accidental implementation
  constraints. A repeated whole-suite callback is not a new focused case.
- Preserve the original native baseline before migration. For every file, inspect
  the complete diff, verify independent expected results and required failure/alias
  frames, exercise affected scalar/backend/macro profiles and negative controls,
  then commit. Format and Release assertions must remain active.
- Numerical comparisons need their actual fixture domain and error rationale.
  Unsupported uniform error bounds, unreachable native states and source-only
  obligations remain explicit dispositions; a green native suite cannot close them.
- Current progress is Pending or Implemented. Implemented means code and targeted
  validation are present, not that every requirement is Approved or every target
  qualified. Git owns revision history; local execution logs stay outside this file.

## Per-file actions

| Action | Primary file | Work / review boundary | Progress |
|---|---|---|---|
| T-01 | `tests/CMakeLists.txt` | Centralize test registration; preserve target names and executable paths; enable optimized assertions. | Implemented |
| T-02 | `tests/support/test_support.hpp` | Count assertions and preserve nonfatal failure reporting. | Implemented |
| T-03 | `tests/support/test_backend.hpp` | Select and identify actual backend; verify allocation-guard assumptions. | Implemented |
| T-04 | `tests/support/test_cases.hpp` | Adopt selectable IDs/profiles; reject empty, duplicate, unknown and assertion-free cases. | Implemented |
| T-05 | `tests/tools/test_case_runner.cpp` | Exercise runner success, failure, bad selection and false-pass negative controls. | Implemented |
| T-06 | `tests/common/test_contracts.cpp` | Status meanings, floating-point profile and independent-instance checks. | Implemented |
| T-07 | `tests/common/test_compile_contracts.py` | Compile-time shape/configuration restrictions, dependency boundaries and explicit inspection dispositions. | Implemented |
| T-08 | `tests/math/test_scalar.cpp` | F-SCALAR: checked primitives, aliases, domains, result failures and independent known answers. | Implemented |
| T-09 | `tests/math/test_linalg.cpp` | F-LINALG: values, access, dimensions, arithmetic, reductions, normalization and derived operations. | Implemented |
| T-10 | `tests/math/test_spd_solve.cpp` | F-SOLVE: equations, correlated/multiple RHS, domains, failures and aliases. | Implemented |
| T-11 | `tests/math/test_so3.cpp` | Quaternion and Exp/Log: coefficients, geometry, branches, domains and failure frames. | Implemented |
| T-12 | `tests/math/test_so3_jacobian.cpp` | Right Jacobian formula, branch boundary, independent derivative and failure frames. | Implemented |
| T-13 | `tests/math/test_cmsis_dsp_backend.cpp` | Qualify adapter-specific storage/padding checks; retain only contract-relevant assertions. | Implemented |
| T-14 | `tests/core/test_types.cpp` | State/configuration dimensions and field/coordinate ownership. | Implemented |
| T-15 | `tests/core/test_prediction.cpp` | Nominal prediction model, parameter/prior boundaries, consumption, overflow and aliases. | Implemented |
| T-16 | `tests/core/test_covariance.cpp` | Process noise, full transition/noise mapping, publication and joint prediction transaction. | Implemented |
| T-17 | `tests/core/test_injection_reset.cpp` | Injection, reset and three-output transaction, both reset models and all allowed aliases. | Implemented |
| T-18 | `tests/core/test_correction.cpp` | Independent gain/Joseph equations, solve/failure stages, bias controls and joint publication. | Implemented |
| T-19 | `tests/core/test_innovation.cpp` | Componentwise ratios, threshold equality, IEEE limits and complete diagnostic publication. | Implemented |
| T-20 | `tests/core/test_position.cpp` | Position selectors, residuals, noise/permissions forwarding and correction transactions. | Implemented |
| T-21 | `tests/core/test_velocity.cpp` | Velocity selectors, residuals, noise/permissions forwarding and correction transactions. | Implemented |
| T-22 | `tests/core/test_accelerometer.cpp` | Gravity observation, fixed preprocessing, domains and wrapper transactions. | Implemented |
| T-23 | `tests/core/test_magnetometer.cpp` | Magnetic observation, selected gain projection, geometry and wrapper transactions. | Implemented |
| T-24 | `tests/core/test_sequence.cpp` | Portable full-core sequences and meaningful interacting macro profiles. | Pending |
| T-25 | `tests/core/test_codegen.cpp` | Generated caller selection and equation checks; no mere dense/generated agreement claim. | Pending |
| T-26 | `tests/runtime/test_ring_buffer.cpp` | All 11 ring-buffer families, including full-backing clearing evidence and legal horizon aliases. | Implemented |
| T-27 | `tests/runtime/test_tilt_alignment.cpp` | Three candidate producer cases; clarify contract binding without inventing requirement approval. | Implemented |
| T-28 | `tests/runtime/gravity/test_magnitude.cpp` | Vector filtering, strict band, history preservation and cadence. | Implemented |
| T-29 | `tests/runtime/gravity/test_sampling.cpp` | Rotating accumulation, interval completion, clipping, failures and source interruption. | Implemented |
| T-30 | `tests/runtime/gravity/test_conditions.cpp` | Complete eligibility truth table and failed evidence. | Implemented |
| T-31 | `tests/runtime/gravity/test_fusion.cpp` | Condition-first correction, permissions, failure frames and SW-GRAVITY sequences. | Implemented |
| T-32 | `tests/runtime/imu/test_bias_inhibition.cpp` | Peak dynamics, axis permissions, failure history and missing-sample behavior. | Implemented |
| T-33 | `tests/runtime/imu/test_process_noise.cpp` | Fault/clipping noise replacement, validation and immediate restoration. | Implemented |
| T-34 | `tests/runtime/imu/test_bias_process_noise.cpp` | Bias-noise thresholds, consumption, aliases, inhibit/release and SW-IMU sequence. | Implemented |
| T-35 | `tests/runtime/imu/test_bias_limits.cpp` | Per-axis limits, complete validation and idempotent/released clamping. | Implemented |
| T-36 | `tests/runtime/imu/test_vertical_health.cpp` | Signed evidence, source grouping, clipping counters, clocks, reset and failure frames. | Implemented |
| T-37 | `tests/runtime/gnss/test_quality.cpp` | Quality flags, thresholds, filters, qualification, invalid calls and history. | Implemented |
| T-38 | `tests/runtime/gnss/test_lifecycle.cpp` | All 15 lifecycle families and portable SW-GNSS command/feedback sequence. | Implemented |
| T-39 | `tests/runtime/gnss/test_measurement.cpp` | Generic/named gated fusion, diagnostics, vertical recovery and sequential groups. | Implemented |
| T-40 | `tests/runtime/gnss/test_reset.cpp` | Four reset equations, covariance masks, delta, aliases and failure publication. | Implemented |
| T-41 | `tests/runtime/test_measurement_fusion.cpp` | Generic measurement reporting, checked gate/correction and failure transactions. | Implemented |
| T-42 | `tests/runtime/magnetic/test_field.cpp` | Independent field geometry, selection/reference combinations, thresholds and failures. | Pending |
| T-43 | `tests/runtime/magnetic/test_reference.cpp` | Reference conversion/adoption/timing; revision exhaustion mapped to scoped source evidence. | Implemented |
| T-44 | `tests/runtime/magnetic/test_heading.cpp` | Acceleration/heading filters, observability, latch, partial resets and failures. | Pending |
| T-45 | `tests/runtime/magnetic/test_filter.cpp` | Reset-only filter seed/recurrence, failure atomicity and source/reference distinction. | Pending |
| T-46 | `tests/runtime/magnetic/test_fusion.cpp` | Field/gate/correction stages, independent references, effective gain and diagnostics. | Pending |
| T-47 | `tests/runtime/magnetic/test_reset.cpp` | Heading/covariance transport, delta quaternion, floors, failure frames and repeat behavior. | Pending |
| T-48 | `tests/runtime/magnetic/test_lifecycle.cpp` | All 13 lifecycle families and SW-MAGNETIC source/reference/reset sequences. | Pending |
| T-49 | `scripts/verify_tests.py` | Reconcile design IDs/dispositions with registrations; run cases with per-item `[Pass]`/`[Failed]`, totals and a failing exit status; missing cases cannot silently pass. | Implemented |
| T-50 | `tests/tools/test_test_plan.py` | Negative controls for missing/duplicate/unknown registrations, failed/empty execution, display totals and unsupported evidence claims. | Implemented |
| T-51 | `tests/CMakeLists.txt` | Enforce the declared strict floating-point test profile, including optimized builds. | Implemented |
| T-52 | `tests/support/correction_reference.hpp` | Share an independent dense correction oracle across core/runtime cases; validate with exact analytic controls. | Implemented |
| T-53 | `tests/support/correction_reference.hpp` | Extend the independent oracle with explicit heading gain projection; verify exact identity and tilted controls and default preservation. | Implemented |

## Final reconciliation and cleanup

After these replacements are reviewed, reconcile every design row and every old
core/runtime test in both directions. Keep an old regression with a distinct
requirement/defect sensitivity; relocate or revise it under its owning file.
Delete an obsolete source only when its meaningful observations have a reviewed
replacement. Record necessary retained-test changes in the affected action before
editing. Do not remove integration, replay, tool or proof fixtures merely because
they are outside the 250-row design.

Run the complete supported native selection, optimized compiler checks and
ASan/UBSan; exercise the documented macro interactions and available backend
adapters. Verify removed paths have no live CMake/script/spec/traceability caller.
Report implemented cases separately from partial numerical/source-evidence rows,
unsupported profiles and remaining retention decisions. No test-count percentage
stands for correctness, structural coverage or proof completion.
