# Bounded runtime storage and alignment tests

**Status:** Draft

**Scope:** Portable fixed-capacity `RingBuffer` and the exposed `TiltAlignment`
operation. IMU-history owners, delayed observation/context queues, output
predictors, height owners and module/replay wiring are outside this design.

**Level and purpose:** Storage unit/sequence tests, compile-time API checks and
bounded-resource inspection; candidate alignment unit cases with an explicit
requirement-ownership gap. Existing tests are not used as design input.

**Profiles:** Follow [shared profiles and oracle rules](../profiles.md).
Storage is scalar/backend-independent; use capacities 1, 2, 3, 8 and 257, and
active lengths 1, less than capacity and capacity where distinct. Samples contain
nonzero uint64 timestamps, distinct payloads and an allocation-free bounded
copy-counting type where useful. Alignment uses binary32/binary64 and supported
backends with caller-established unit q and symmetric PSD attitude covariance.

**Setup and oracle:** A host list model stores ordered timestamp/payload pairs,
trims the oldest on full push, and selects the newest eligible timestamp at pop.
It does not reproduce ring indices or the production backwards scan. Independently
map logical entries to physical slots for access checks. Before mutation save
all observable payloads/counters/output; only contract-preserved fields must
remain exact. Alignment reference uses independent 3D rotation and covariance
equations. No tests invoke empty endpoint views, invalid indices, nonchronological
pushes or storage-aliased output and then invent an expected error response.

**Regression:** Deterministic wrap/fill/pop/reconfigure sequences and threshold
fixtures are stable regression cases. Every case has an explicit fault target;
the case tables identify executable bindings and remaining acceptance gaps.

## Operation allocation

| Production header / public group | Cases / disposition |
|---|---|
| `ring_buffer.hpp`: explicit construction, deleted default/copy/move operations, allocate/valid | `TC-R-RING-BUFFER-01`, `02` |
| `ring_buffer.hpp`: push, pop_first_older_than | `TC-R-RING-BUFFER-03` through `07`, `10` |
| `ring_buffer.hpp`: get_oldest/get_newest, const/mutable operator[], get_oldest_index, entries/get_length, used/total byte queries | `TC-R-RING-BUFFER-08`, `09` |
| `ring_buffer.hpp`: reset; bounded storage/work contract | `TC-R-RING-BUFFER-02`, `10`, `11` |
| `tilt_alignment.hpp`: construction, update, aligned/variance and public tilt_limit | `TC-R-ACCELEROMETER-CONDITIONS-03` through `05`, blocked for requirement approval as described below |

## Fixed-capacity ring buffer

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and independent oracle | Executable test |
|---|---|---|---|---|
| TC-R-RING-BUFFER-01 | [R-RING-BUFFER](../../../requirements/runtime/ring-buffer.md), construction/type restrictions | Construct with lengths 0,1,Capacity,Capacity+1; compile traits for default/copy/move construction/assignment; compile-fail Capacity=0 separately. | Valid only for permitted nonzero lengths; invalid construction empty/invalid. Default/copy/move unavailable as specified. No runtime invalid-index probe. Detects unrestricted copying or unsafe initial length. | Partial: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp); capacity-zero compile check belongs to the shared build profile |
| TC-R-RING-BUFFER-02 | Same, allocate and reset | Populate/wrap buffer; allocate invalid length, same length and changed valid length; reset nonempty/empty valid buffer. Inspect legal active slots. For full backing use a bounded Sample assignment observer or source proof recording all Capacity distinct slots cleared. | Invalid/same allocation preserves all state; changed length and reset clear full backing history/indices while preserving selected length. Reset permits a new ordered timeline. Active-slot checks cannot establish clearing of inaccessible backing slots; expanding later also resets and would mask an earlier defect. Separate clearing evidence detects that omission. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-03 | Same, push/overwrite and chronological invariant | Push 0..3×active-length ordered samples with distinguishable payloads, including capacity1 and non-power-of-two active lengths. | Last min(count,length) samples retained in order; full push overwrites exactly oldest, entries bounded, physical oldest/write progression correct. Caller inputs pure; no rejection invented for caller-owned timestamp admission. Detects off-by-one wrap and power-of-two assumptions. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-04 | Same, newest eligible selection | Empty/single/multiple/wrapped history; horizon before oldest, exactly a sample, between samples and after newest. Choose age window keeping the selected sample valid. | Pop selects newest sample≤horizon, not oldest candidate; retires selected and all older, preserves future payloads/order; misses preserve output. List oracle catches reversed search and consuming future data. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-05 | Same, strict max age and miss retirement | Candidate age window−1, equal and +1; zero window, explicit window and omitted default100000 us; all older expired with future entries retained. | Strict age<window; equality misses but retires expired prefix; output sentinel unchanged on miss. Entry/slot timestamps show retired prefix while payload clearing is not required by pop. Detects inclusive age and assuming every false return preserves history. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-06 | Same, null output frame | Pass null output on empty/nonempty/full/wrapped buffers with horizons that would otherwise select or expire entries. | false and entire buffer unchanged; compare a subsequent ordinary pop with untouched control. Detects retiring history without a receiving object. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-07 | Same, timestamp subtraction/borrowed horizon | Ordered timestamps near UINT64_MAX, horizon before/at/after candidate, large window. Also pass horizon by reference to disjoint output.time_us and to a valid buffer slot.time_us; output itself stays outside storage. | Selection uses entry horizon and ordered subtraction without overflow; permitted mutations of output/retired slot cannot change the decision midway. No invented promise that an aliased horizon's stored value survives output/retirement writes. Detects timestamp-addition overflow and unsafe rereading after mutation. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-08 | Same, endpoint/physical-slot access | After wraps/pops, read nonempty oldest/newest repeatedly; compare const/mutable physical slots at legal indices; change payload only in a live slot and then pop it. | Views expose correct sample without consuming; payload edit is visible, timestamps/order unchanged. Do not retain a view across mutation. Detects treating physical index as logical position or copying stale payload. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-09 | Same, size/reporting semantics | Each occupancy from empty to full, active lengths different from backing capacity, including257; read length/entries/oldest-index/used-size/total-size. | Size_t counts support >255; total=sizeof object, used=metadata+entries×sizeof Sample; active length does not resize backing storage. Queries pure and consistent with list/physical oracle. Detects uint8 truncation and claiming logical usage equals allocated memory. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-10 | Same, combined bounded operation sequences | Enumerate bounded push/pop/reset/allocate sequences at small capacities; use deterministic longer seeded sequences at larger capacities with legal timestamp/alias premises. Compare after every operation, including empty→full→wrap→empty transitions. | List model and ring invariants agree at every step; oldest/newest and all live payloads match, inactive time tags zero, count/index bounded. Paired known-answer cases above prevent a mutually wrong model. Detects interaction defects missed by isolated operations. | Native: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp) |
| TC-R-RING-BUFFER-11 | Same, fixed storage and bounded work | Compile/inspect instantiated buffer operations, instrument bounded payload copies, and run host allocation detection across allocation/push/pop/reset. Inspect loops and forbidden dependencies. | No heap allocation/locks/logging in buffer operations; push bounded O(1), pop/reset at most capacity-dependent bounded work; stored Sample obeys its own bounded-copy contract. Source evidence, not host wall time, supports complexity; target WCET/stack budget remains separate. Detects accidental dynamic storage or unbounded work. | Partial: [test_ring_buffer.cpp](../../../../../tests/runtime/test_ring_buffer.cpp); bounded-copy observations and source inspection; host heap instrumentation belongs to the shared resource profile, no target WCET claim |

## Alignment ownership and candidate acceptance

`TiltAlignment` is an exposed portable API, but its behavior currently appears
in a proof/evidence paragraph appended to
[R-ACCELEROMETER-CONDITIONS](../../../requirements/runtime/gravity/conditions.md),
the requirement's separate stateless eligibility predicate.
The following cases retain this visible behavior for review; they do not create
new normative requirements. Before approving these cases, put the alignment
producer's contract in an explicit requirement and rebind their IDs/trace links
without losing review history. Current default values/public constant are source
observations requiring that approval, not facts inferred from existing tests.

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and independent oracle | Executable test |
|---|---|---|---|---|
| TC-R-ACCELEROMETER-CONDITIONS-03 | R-ACCELEROMETER-CONDITIONS, alignment evidence paragraph; ownership pending | Construct; query aligned/variance; update identity and tilted unit q with anisotropic PSD P and nonzero cross terms; re-read accessors. | Candidate contract: initial false latch/zero diagnostic; variance is N/E diagonal sum of independent `R P R^T`; queries pure. Protects against body-frame diagonal sum and unintended alignment merely from default zero variance. | Partial: [test_tilt_alignment.cpp](../../../../../tests/runtime/test_tilt_alignment.cpp); candidate producer contract ownership remains pending |
| TC-R-ACCELEROMETER-CONDITIONS-04 | Same, strict threshold and sticky latch | Independently construct variance below/equal/above encoded `(pi/60)^2` where reachable; above→below→above sequence; reconstruct new owner. | Candidate contract: strict inequality acquires once, later high variance updates diagnostic but not latch; new object clears latch. No public reset method is assumed. Detects inclusive boundary and accidental unlatching. | Partial: [test_tilt_alignment.cpp](../../../../../tests/runtime/test_tilt_alignment.cpp); candidate producer contract ownership remains pending |
| TC-R-ACCELEROMETER-CONDITIONS-05 | Same, failure frame | Nonfinite consumed P, finite rotated arithmetic overflow, negative computed N/E diagonal through an explicitly admitted invalid-input profile; malformed q through public underflow/multiplication fixtures in the declared host profile. | Candidate contract: exact nonfinite/domain status, both diagnostic/latch unchanged on failure, borrowed inputs pure. Symmetric PSD/unit q remain success premises. Native tests do not bypass UnitQuaternion construction. Detects diagnostic publication before validation. | Partial: [test_tilt_alignment.cpp](../../../../../tests/runtime/test_tilt_alignment.cpp); candidate producer contract ownership remains pending; malformed values use public construction and multiplication |

## Review Pending

- Approve or revise the alignment requirement owner and construction/reset
  semantics before treating its candidate cases as normative acceptance.
- Review the listed fixture/selector bindings and required profiles before
  accepting results; these documents record design and scope, not executions.
- Bounded copy/work inspection does not certify a deployment's WCET, total
  memory budget, single-owner enforcement or caller loss-handling policy.
