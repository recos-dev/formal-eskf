# R-RING-BUFFER: Fixed-capacity timestamped history

**ID:** `R-RING-BUFFER`

**Status:** Draft

**Applies to:** Portable, single-owner runtime storage; payload and timestamp premises below.

## Basis

The reference is pinned [PX4 RingBuffer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/RingBuffer.h#L48).
The API uses its operation names and non-consuming history access, but is not a
binary-compatible replacement: capacity is bounded at compile time, lengths/counts
use `std::size_t`, and the optional third selection argument permits an explicit
source age window. Consumed/expired timestamp tags are cleared, so `entries()`
counts active retained entries, unlike PX4's scan of all nonzero physical slots,
which can include superseded data. Expired entries are retired even on a miss;
PX4 may retain them. No PX4 source is vendored.

## Required behavior

`include/formal_eskf/runtime/ring_buffer.hpp` provides the portable
`formal_eskf::runtime::RingBuffer` with the PX4-style history API and fixed backing
storage. `RingBuffer<Sample, Capacity>(length)` reserves `Capacity`
slots inside the object and selects an active length in `1..Capacity`. It is a
single-owner runtime utility with no PX4, clock, scheduler or linalg dependency.
Sample construction/copying must be bounded, allocation-free, non-throwing and
non-blocking, with `std::uint64_t time_us` in the common microsecond time domain.
Default construction, copying and moving the buffer are disabled, as in PX4.

| Operation | Required behavior |
|---|---|
| Constructor, `allocate(length)`, `valid()` | Invalid initial length leaves an invalid empty buffer. `allocate` selects a length within the backing capacity without heap allocation; invalid requests leave everything unchanged, equal length preserves history, changed length resets it. Reconfiguration is startup/restart only. |
| `push(sample)` | Requires a valid buffer and caller-admitted, strictly increasing nonzero timestamps. Return `void`, retaining the newest sample and overwriting the oldest when full, as in PX4. It does not validate source timestamps or report overflow. |
| `get_oldest()`, `get_newest()` | Non-consuming const references; require `entries() > 0`. Saved views must not be used across buffer mutations. |
| `operator[](index)`, `get_oldest_index()` | Physical slot access, with `index < get_length()`. The owner may correct payloads while preserving their timestamps and ordering. Traverse retained entries from the oldest index with wraparound; zero-time slots are inactive. |
| `pop_first_older_than(timestamp, output, max_age_us = 100000)` | Return `bool`, writing the newest retained sample at/before the horizon with age strictly below the window. The two-argument pointer call and default 100 ms window match PX4. Output must be outside buffer storage. Retire selected/older entries and preserve future ones. On a miss, expire old entries without changing output; null output returns false without mutation. |
| `get_length()`, `entries()` | Report configured length and active retained count. Count/indices use `std::size_t`, avoiding PX4's 8-bit capacity limit. |
| `get_used_size()`, `get_total_size()` | Report metadata plus active payload bytes, and the complete object size including all backing storage. Unused storage remains reserved; no memory is released on consumption. |
| `reset()` | Clear all backing slots, occupancy and indices while retaining the configured length. It does not allocate. |

### State and preservation

Required properties:

- Construction establishes empty history. Invalid allocation preserves every
  field; unchanged length retains history; changed length and reset clear all
  backing slots. Reset retains the configured length.
- Valid histories have bounded count/indices, consecutive active slots in ring
  order, strictly increasing nonzero active timestamps and zero inactive tags.
  The next write slot follows the retained sequence, including the full case.
- An admitted push appends exactly one sample, overwriting only the oldest when
  full. Other slots and the borrowed input remain unchanged.
- Selection returns the newest sample at/before the horizon with age strictly
  less than the window, consumes the at-or-before prefix and preserves future
  samples. A miss preserves output; null output preserves the entire buffer.
  Ordered age subtraction must remain safe across the full `uint64_t` domain.
- Endpoint/count/size/index queries are non-consuming. Indexed payload writes
  retain timestamps, other payloads, occupancy and order. Empty endpoint access,
  out-of-range indexing and an output alias into storage are outside the contract.

### Bounded execution

Push, endpoint/index/count/byte queries take constant work for the selected sample
type. Selection examines at most `Capacity` timestamps and clears at most
`Capacity` tags, copying at most one sample. Reset/changed-length configuration
clear at most `Capacity` samples. Length changes are startup/restart only;
source/history invalidation may reset during a callback, whose budget must include
that bounded clearing cost and the associated owner reset work.
No operation allocates, locks, logs or resizes backing storage. Source capacities, timing and target memory/latency budgets belong to the
deployment profile.

## Acceptance and verification

The ESBMC scope is the actual header instantiated with a zero-default sample
containing `uint64_t time_us` and one arbitrary `uint64_t` payload, with explicit
fieldwise copy operations. Profiles use capacities 1, 3 and 8, every active length
within each capacity, and both 32- and 64-bit word models. Timestamps, payloads, initial output, horizon and age window
are symbolic; there is no small-time or successful-selection premise. The default
two-argument selection has a separate one-sample check of its default-window
binding; the general selection check quantifies every explicit window. Null output
is checked with the non-consuming access operations. These profiles use a separate
horizon object; horizon references aliased into output or buffer storage remain
outside this proof scope, without adding an API restriction.

Passing initialization and arbitrary invariant-state operation checks discharge
the base/preservation obligations for histories of any operation count within
those instantiations. All registered profiles must finish successfully; timeouts
and incomplete unwinding leave a gap. This is not merely a fixed push/pop
sequence, nor a proof for every template capacity or payload type. The harness exposes private state only
to seed and check the invariant; no production operation is replaced. Seed
metadata uses 8-bit integers covering every valid index/count in these capacities;
production fields retain their actual types. An independent arbitrary slot index
checks all storage positions universally. A fieldwise oracle snapshot avoids ESBMC 8.4's aggregate-copy conversion limitation; the production
array still uses ESBMC's standard-library model. Native tests supply independent
linear-list and IMU/observation/output caller scenarios.

The caller must provide single ownership, ordered source admission, non-throwing
bounded payload operations, loss handling and appropriate configuration phases.
Monotonic fusion horizons, delayed-IMU continuity, output-epoch alignment and
module integration remain separate obligations. These integer/storage checks
need no real-number Lean model; they do not verify deployed standard-library
internals, target layout, every payload type, scheduler behavior or target WCET.

Native evidence is `tests/runtime/test_ring_buffer.cpp`, registered as
`formal_eskf.runtime.ring_buffer`: configuration/failure preservation, non-consuming
history, wrap/overwrite, indexed payload correction, selection/expiry/future
retention, timestamp boundaries and deterministic sequences against a linear-list
model. Caller fixtures exercise IMU warmup/once-only prediction/history loss,
source admission/late arrival/qualification/calibration restart, and output
timestamp matching/history correction/reset through the production buffer API.
Those fixtures demonstrate the usage contracts; they do not implement or verify
the module's admission, delayed filter or output predictor. Package tests check
delivery through the shared headers. This is component regression evidence,
not deployed integration, a formal proof or target WCET.
