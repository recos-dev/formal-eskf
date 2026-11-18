# PX4-RT: real-time constraints

**ID:** `PX4-RT`

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

The estimator path must have bounded work and storage. It must not wait for shell
diagnostics, perform blocking console/file/network I/O, sleep, busy-wait, resize
containers or allocate dynamically during normal operation. No C++ exception
may propagate through module execution, including dependency failure paths.
Build flags alone do not establish this contract.

Diagnostic readers consume a fixed-size snapshot. Snapshot exchange must avoid
data races and have a bounded writer cost; an unavailable diagnostic exchange
may skip a snapshot, never an estimator update. Formatting and printing happen
outside estimator synchronization. Snapshots carry their epoch/sequence so a
skipped update is visible as stale diagnostics. This is deliberately stronger
than copying PX4's direct diagnostic reads or holding a filter-wide mutex through
printing.

Startup allocation is permitted with explicit failure handling. Audit platform
dependencies too: uORB `advertise()` alone does not guarantee that first-publish
storage exists. Complete required first-use preparation before normal operation,
without publishing fabricated valid estimates. Use bounded platform primitives
where needed and account for their locking and callback costs. Do not replace
missing allocation/error handling with `noexcept` annotations.

Large numerical temporaries require a compiler stack report and full call-chain
analysis or explicit per-instance workspace. Never introduce shared global
scratch storage. The current 60 KiB SITL reservation is not a target budget or a
stack bound for ARM/STM32.

Each deployable profile records the values below. Validate parameter domains and
cross-constraints at startup; establish execution bounds through target evidence,
not configuration checks inside every estimator update.

| Profile data | Required relationship |
|---|---|
| Supported IMU arrival rate, integration limits and gap tolerance | Define the input envelope; nominal frequency alone is insufficient |
| Main-filter period, fusion delay, per-source delay/age/rate limits | Preserve measured integration periods and keep accepted samples within the history span |
| History capacities and memory/stack limits | Cover the admitted rates, retention and deepest supported call path without resizing |
| Queue priority, attitude/output-age limits and cycle deadline | Account for scheduler interference, synchronization, computation and publication |
| Scalar/backend, algorithm switches, numerical and source parameters | Identify one reproducible configuration with validated units/domains |

The attitude response budget includes scheduling wait, fast propagation and
publication. The complete callback budget includes any main-filter step, all
eligible aiding, feedback and diagnostics. Both must fit their declared deadlines;
the cycle deadline must fit the supported minimum IMU inter-arrival interval.
Measure the simultaneous-aiding, reset, outage and fault paths, not only the
average prediction tick. The same work queue cannot preempt its own slow phase.

Retain wall-clock response/CPU measurements, missed messages/deadlines, maximum
output age, stack high-water and allocation/synchronization observations for
the declared target and load. A measured maximum is an observed bound, not an
analytical WCET proof. Lockstep simulation time and percentile statistics alone
do not establish hard-real-time acceptance.
