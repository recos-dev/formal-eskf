# C-LAYER-BOUNDARY: Layer ownership and dependency boundaries

**ID:** `C-LAYER-BOUNDARY`

**Status:** Draft

**Applies to:** Shared numerical core, portable runtime and deployment adapters.

## Basis

Separate numerical behavior from policy and platform effects so each boundary can be verified independently.

## Required behavior

| Layer | Responsibility | Must not own |
|---|---|---|
| Numerical core | Prediction, correction, injection/reset and covariance | Clocks, queues, sensor policy or OS effects |
| Portable runtime | Qualification, history, lifecycle and fusion decisions | Device access, scheduling or publication |
| Platform adapter | Message/time conversion, session ordering and output | Undocumented changes to core equations |

Keep numerical APIs independently callable and backend/platform effects behind
their declared boundaries. These are responsibilities, not a required class
hierarchy or a generic dispatch framework. Portable runtime has no single object
that owns and schedules an entire estimator for every platform.

## Acceptance and verification

Inspect public interfaces, dependency direction and state ownership. Core and
portable-runtime code must not acquire the platform effects prohibited by the
table. A component proof does not establish the adapter's timing or transactions.
