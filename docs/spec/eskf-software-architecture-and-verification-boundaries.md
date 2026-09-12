## Summary

Define the software architecture and verification approach before extending the ESKF runtime or integrating it into PX4. The focus is ownership, control flow, lifecycle, time, failure handling and bounded resources, rather than Kalman or quaternion algebra.

This extends [Formal Verification Strategy](https://github.com/shengwen-tw/formal-eskf/issues/1) and follows the existing [C++ portability constraints](https://github.com/shengwen-tw/formal-eskf/issues/6).

## Architecture

| Layer | Responsibility | Must not own |
|---|---|---|
| Numerical core | Prediction, correction, injection and covariance operations | Clocks, queues, sensor selection or OS effects |
| Portable estimator runtime | Lifecycle, event ordering, fusion decisions and coherent estimates | Device access, OS scheduling or publication |
| PX4 adapter | Message conversion, time sources, serialized access and publication | Undocumented changes to core equations |

Keep the numerical functions independently callable. The runtime is an optional consumer, not a replacement that forces every application into PX4's scheduling model. These are responsibility boundaries, not a prescribed class hierarchy.

Use explicit ownership and isolate external effects in the adapter. A single-writer runtime is the proposed baseline; integration must establish that assumption rather than treating the sequential proof as a concurrency proof.

## Software proof obligations

Use Lean to establish abstract state-transition invariants and ESBMC to check production C++ behavior under explicit assumptions and bounds. Review their correspondence separately; agent review is not a machine-checked proof bridge.

| Area | Property to establish |
|---|---|
| Ownership and memory | Valid lifetimes and accesses under the declared ownership model |
| Lifecycle | Initialization and recovery follow legal transitions |
| State updates | Success is coherent; rejection/failure preserves the estimate |
| Time and context | Fusion respects alignment, identity and coordinate/configuration rules |
| Output | Published estimates have consistent state, time and validity |
| Resources and progress | Operations stay bounded; progress claims state their environment assumptions |

## Review and development gates

| Gate | Required outcome |
|---|---|
| Architecture | Agree on responsibilities, ownership and side effects |
| Behavior | Define operation contracts and lifecycle rules |
| Environment | Identify integration assumptions and supported configurations |
| Proof design | Identify invariants, dependencies and applicable evidence |
| Implementation | Develop components together with their verification |
| Acceptance | Review actual evidence and keep unresolved gaps visible |
