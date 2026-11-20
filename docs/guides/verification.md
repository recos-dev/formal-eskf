# Verification scope and evidence

This guide explains what the existing evidence can establish and how to review
it. It is not a live pass report. Requirements belong to [spec/](../spec);
[traceability-map.json](../../formal/agent-review/traceability-map.json) maps sources,
Lean declarations and ESBMC harnesses. A reference or inventory row is not an
executed proof. Consult the [review criteria](../spec/verification/assurance.md)
for acceptance and the [roadmap](../roadmap.md) for remaining work.

The [verification strategy](../spec/verification/assurance.md) defines the
separate claims of Lean, ESBMC, native/static checks and agent review. Some source
proofs use actual producers plus caller summaries; all matching dependencies and
profiles must be accepted together. The AST inventory discovers declarations;
it is neither a proof nor a mandatory review of every function.

## Open boundaries

- Full `HeightAiding` transactions remain outside the completed producer checks.
- Live IMU/PX4/replay calling sequences still need source-composition evidence.
  IMU, observation and output-owner contracts and induction are described below;
  their component evidence and native module tests do not prove the live sequence.
- Deployed matrix/libm accuracy, numerical error budgets and repeated-step
  invariants are not implied by finite outputs or exact-real theorems.
- Work-queue/consumer behavior, dependency allocation/locking, forced shutdown,
  full stack/response-time bounds and physical sensor/model validity require
  target/system evidence. SITL and recorded-log replay do not qualify hardware.
- AST-to-requirement baselining and one-result-per-symbol completeness remain V-03;
  the existing extractor does not enforce them.

The current module's functional boundaries are in [the integration guide](../../integrations/px4/README.md),
[PX4 timing](../spec/profiles/px4-sih-250hz.md) and the
[functional roadmap](../roadmap.md#functional-work). The SIH contact-model failure
tracked in F-07.1 is not
converted into a passing landing result by excluding the failed scenario.

The [INS codegen package](../../codegen/README.md) is selected by default through
`ESKF_CODEGEN`. It has separate exact-algebra, conditional numerical and
memory-safety checks. Production selection is tested by CTest; differential
fixtures alone do not cover it. Existing ESBMC core producer/caller profiles are
explicitly dense (`ESKF_CODEGEN=0`) except the explicit generated suites below.
Run native suites in both modes.

### Generated covariance prediction

`./scripts/verify_esbmc.sh pcov-generated` selects `ESKF_CODEGEN=1`. Its INS
caller profiles cover both scalar formats, attitude modes and whole-P aliases.
They execute the production caller with unconstrained pure producer returns,
checking complete F/P/R/Q forwarding, validation order, status propagation and
publication/rollback. Separate actual-kernel profiles check input frames and
mirroring on the same backend. Actual `publish_covariance` profiles check the
finite scan, exact copying without averaging and failure preservation; they
include all IEEE candidates and both aliases.

Acceptance also requires the matching `pcov` transition, quaternion-validation,
rotation and INS noise/storage producers, their Exp/normalization dependencies,
and the reused same-type matrix/finite/storage producers. Successful noise
construction supplies diagonal Q_i; the caller checks every F entry. Symmetric
PSD P remains a caller premise. Neither kernel nor publisher summaries assume
success, finiteness or symmetry. Exact kernel equations are covered by the
separate codegen algebra evidence, not by these unconstrained-return summaries.

### Generated reset, correction and transactions

`reset-generated` covers the actual INS reset caller in both reset modes and
both whole-P alias arrangements, with the same right-Jacobian/finite producers
as `reset`. Its kernel profiles check complete input frames and mirrored output;
it reuses the actual publisher from `pcov-generated`.

`correction-generated` separates three obligations for each explicit observation
layout: actual cross/innovation/Joseph kernel frames, actual dispatch forwarding,
and checked preparation/gain/Joseph callers. Body-vector dispatch must extract
the three correct skew-Jacobian entries. Joseph receives the full effective gain
and full correlated V; it does not assume an unconstrained optimal gain. Known
layouts require their exact H, supplied by the observation model/wrapper proofs.
Symmetric P/V and the structured A = I - KH are premises of the exact kernel
identities, not assumptions used to hide caller failures.

The caller summaries return arbitrary IEEE matrices and statuses. They are
paired with the existing exact-type validation, solve, matrix-product, subtraction,
finite and innovation-finalizer producers, plus an actual same-backend generated
publisher. Generated Joseph publication copies the mirrored result without a
second average. Logical stage counters in the common caller oracle describe
algebraic stages; they are not counts of executed dense operations.

`step-generated` checks production INS prediction, injection/reset and full
correction transactions, including state/P/error aliases and symbolic bias
permissions. Same-backend reset kernel and actual reset-frame profiles discharge
the terminal component's input/rollback contract in both reset modes. Injection,
observation model/wrapper, bias-mask and solver dependencies remain mandatory;
a component summary alone is not a closed composition proof. AHRS and arbitrary-H
products retain their dense producer obligations.

These are source behavior claims on the fixed-array proof backends. Exact-real
equivalence comes from the independent codegen algebra/Lean certificates;
finite-record IEEE error certificates do not establish a uniform deployment
error budget, repeated-step PSD or identical dense/generated failure boundaries.
Target numerical budgets and deployed backend/library internals remain separate.

## Running checks

From the repository root, choose checks affected by the change:

```bash
./scripts/verify_lean.sh --cache --wfail
./scripts/verify_esbmc.sh --list
./scripts/verify_cppcheck.sh
./scripts/verify_asan.sh
./scripts/format_cpp.sh --check
./tests/test_esbmc.sh
./tests/test_agent_review.sh
```

`--list` inventories scheduled profiles; it does not run them. The two shell
tests use offline fixtures by default. For real source checking and agent review,
consult each runner's `--help` and the review criteria; do not launch the entire
ESBMC suite for an editorial change. No formatting command above fixes source.

## Record ownership and reuse

Keep development logs, trial configurations and per-run investigation notes in
ignored local output. They are useful for local tracking, not standing project
specifications. A durable design decision retains only its reason, affected
contract and unresolved limitation; do not copy the experiment diary into Git.

An accepted result needs source/dependency/tool identity, configuration, exact
command, domain/assumptions, outcome and retrievable artifacts. A release must
retain that evidence outside disposable local output, for example in its CI or
release artifact bundle. Documentation links alone do not supply missing logs.
A dirty-tree run needs a snapshot identity, not a claim about unchanged HEAD.

Reassess changed sources, assumptions, models and configurations before reusing
results. Keep failed acceptance cases and counterexamples visible in the relevant
issue/analysis. Missing execution, timeout, OOM or incomplete unwinding stays a
gap. Printed ESBMC programs lose information and are not an equivalence oracle;
do not revive the withdrawn text-based deduplication optimization.

## Runtime producer boundaries

The `height` suite includes per-producer history-domain preservation, complete
innovation-reporting frames and the actual double-precision reporting filter.
The bias producer also admits non-finite scratch variance; it proves the stated
finite-result bounds without assuming successful owner publication. These
properties do not close the `HeightAiding::update` transaction.

`delayed-input` checks the implemented 250 Hz signed timing configuration,
capacity/arrival budget, unchanged output on rejection, and binary32/binary64
input admission using the actual mean-rate and clipping operations. Its scoped
three-value unsigned maximum model has a separate relational check. It does not
prove the history's seed/push/completion or retained payloads. INS in-place nominal
failure frames are checked in `step`; the matching quaternion producer chain is
still required before using that contract in an output-owner proof.

`delayed-views` checks IMU latest/delayed views, output delayed/current/feedback
views, readiness and scalar metadata with the actual 64-slot payload types on
the fixed-array proof backend, binary32/binary64 and the 64-bit word model.
Every valid ring-metadata tuple is
admitted; payloads and timestamps remain arbitrary. This establishes the view
contract under that premise, not that every history mutation preserves it.
No producer body is summarized by these checks; the startup maximum model is
not executed. Full payload mutation frames, history induction, numerical output
tracking and module sequencing remain separate obligations.

`delayed-seed` checks actual IMU construction and seed in binary32/binary64 with
64-bit words and the actual 64-slot storage. Construction initializes every
field. Seed checks startup guards, exact admission arguments/call order, complete
external-input publication, failure rollback and unchanged configuration, fault and ring
metadata. Fieldwise snapshots check every backing slot, including inactive
payloads, without a chronological or finite-value premise. Float frames retain
values, signed zero and NaN classification; they do not claim NaN-payload identity.
The admission summary may return either result and supplies no successful-input
assumption. Acceptance requires the matching actual `delayed-input` producer
profiles; the summary alone does not establish the admitted input domain.
Push, completion, reset and history induction require the `delayed-history`
contracts below; live caller sequencing remains separate. Flat homogeneous snapshots avoid the verifier's conditional-reference
and nested array/tuple encoding limitations without narrowing the checked domain.

`delayed-storage` checks the actual `RingBuffer<ImuHistory::Interval, 64>::push`
on the same proof backend and word model. It checks every field of the inserted
interval, unchanged external input, exact index/count updates and preservation
of every other backing slot, including inactive storage. Valid allocated ring
metadata (active length greater than zero) and a non-aliasing external input are
its premises; payloads and
timestamps remain arbitrary. The frame predicate preserves signed zero and NaN
classification and is separately checked against the existing scalar equality.
All write-position partitions in both scalar formats are mandatory: each retains
the full 64-slot object, and their union covers every valid destination. This
partition avoids the verifier's dynamic aggregate-address expansion; passing a
subset does not discharge the storage contract. IMU owner admission, interval
construction, chronological induction, completion and reset require the
`delayed-history` contracts below.

`delayed-history` checks actual IMU configure/push/complete/reset contracts and
chronological induction separately. It requires the same qualified tool,
64-slot payload, 64-bit word model and both scalar formats for source contracts.
The actual mean-rate producer covers every coefficient and the full input frame.
Typed reset checks all storage values and enclosing-owner frames as separate
obligations over the same unrestricted metadata domain: allocation changes length
before reset, so old indices need not fit the new length.

Push and configuration execute their actual bodies with proved typed-storage or
reset boundaries summarized. Behavior profiles check exact eligibility, first
fault, producer arguments/order, publication and metadata. Frame profiles check
every physical slot before the producer and after return, including inactive
storage. The summary havocs the complete owner, restores every non-buffer field
and supplies exact metadata; caller frames preserve its arbitrary payloads.
Actual typed-producer evidence supplies the missing value equations. Every
partition in both formats is mandatory. Completion and owner reset execute
without these summaries and check their complete frames.

In the pinned ESBMC implementation, `__ESBMC_init_object` havocs the enclosing
root object even when given a pointer to one of its members. A producer summary
using it must restore every required frame of that root, after checking the
caller's pre-state. Passing a member pointer does not limit the havoc to that
member. These assumptions require matching actual-producer evidence.

Borrowed-input profiles execute push using each physical stored input, assuming
its timestamp does not exceed the previous endpoint. Rejection preserves every
stored field. Temporal induction and the actual view contracts discharge this
premise for fresh latest/delayed views; expired views are outside the contract.

The integer-only temporal proof is a relation derived from source contracts,
not another execution of the owner. It retains original integer widths and
unsigned arithmetic. Fault, source, rate and clock decisions are overapproximated
by permitting additional success/rejection transitions. Rejection closes by the
proved temporal frame; no success or chronology is assumed for failed calls.

| Temporal relation | Mandatory source bridge |
|---|---|
| Initialization | Complete constructor initialization in `delayed-seed`. |
| Configure | Exact behavior and every frame, typed reset, timing producer and its maximum model. |
| Seed | Complete publication/frame and actual admission's time/period consequences. |
| Push | Necessary success guards, interval/previous-input forwarding, exact metadata, every caller frame and typed-storage partition. |
| Complete | Exact oldest-interval eligibility, cursor update and complete frame. |
| Reset | Full owner reset, retained configuration and cleared previous input/storage. |

Configure, seed and reset establish empty-history metadata; an independent lemma
checks that every slot clause follows regardless of stored values. Successful
push and completion use arbitrary observed, oldest and newest cells, plus the
retained endpoint. Equal physical indices must yield equal reads, including
predecessor endpoints. Every concrete 64-slot array satisfies these consistency
premises; unrelated cells remain arbitrary. This projection avoids symbolic
array-selection cost without reducing capacity or timestamp ranges.

Common metadata and each chronological slot establish positive ordered durations,
adjacency, newest/previous equality, oldest/completion continuity and full-history
lag. The full pre-invariant entails the observed/oldest/newest premises; a universal
observer checks each post-slot. Assert-then-assume metadata cuts retain their
assertion obligation. Source-to-contract inclusion is an explicitly reviewed
compositional boundary, not an automated whole-owner or Lean/C++ refinement
certificate. All source and temporal dependencies need current successful evidence.
These claims do not establish numerical prediction success, output tracking,
module calling order or WCET.

Large aggregate/reference transactions require scoped tool qualification under
ESBMC 8.4.0. Its released build miscompiles address-of reference-returning calls;
such a counterexample is a tool limitation, not an estimator defect. The IMU
contracts use the qualification below. OOM, timeout and partial property results
remain gaps; do not rewrite production bodies or reduce history capacity to
relabel an obligation as proved.

### Reference-address tool qualification

[reference-address.patch](../../configs/esbmc/reference-address.patch) applies to
ESBMC commit `61172c6f9afe6faee097b117edd0347cfbb6f28d` (v8.4). It preserves the
pointer representing a reference when translating `&reference_returning_call()`;
the unpatched frontend takes the address of the temporary pointer instead.
It changes the verifier, not the estimator or a proof summary.

Build this optional tool separately, using Clang/LLVM 18 development files and
C++23 standard-library headers. The following enables the pinned Z3 and CVC5
dependencies; suites requiring Bitwuzla need that backend built separately.

```bash
git clone https://github.com/esbmc/esbmc.git tools/bootstrap/esbmc-reference
git -C tools/bootstrap/esbmc-reference checkout 61172c6f9afe6faee097b117edd0347cfbb6f28d
git -C tools/bootstrap/esbmc-reference apply "$PWD/configs/esbmc/reference-address.patch"
git -C tools/bootstrap/esbmc-reference apply "$PWD/configs/esbmc/ieee-frame.patch"
git -C tools/bootstrap/esbmc-reference apply "$PWD/configs/esbmc/array-filler.patch"
cmake -S tools/bootstrap/esbmc-reference -B tools/bootstrap/esbmc-reference/build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-18 -DCMAKE_CXX_COMPILER=clang++-18 \
    -DLLVM_DIR=/usr/lib/llvm-18/lib/cmake/llvm -DClang_DIR=/usr/lib/llvm-18/lib/cmake/clang \
    -DDOWNLOAD_DEPENDENCIES=ON -DENABLE_Z3=ON -DENABLE_CVC5=ON -DBUILD_TESTING=OFF \
    -DENABLE_BUNDLE_LIBC_32BIT=OFF
cmake --build tools/bootstrap/esbmc-reference/build --target esbmc --parallel 2
FORMAL_ESKF_ESBMC="$PWD/tools/bootstrap/esbmc-reference/build/src/esbmc/esbmc" \
    ./tests/test_esbmc.sh --references
FORMAL_ESKF_ESBMC="$PWD/tools/bootstrap/esbmc-reference/build/src/esbmc/esbmc" \
    ./tests/test_esbmc.sh --frames
FORMAL_ESKF_ESBMC="$PWD/tools/bootstrap/esbmc-reference/build/src/esbmc/esbmc" \
    ./tests/test_esbmc.sh --initialization
FORMAL_ESKF_ESBMC="$PWD/tools/bootstrap/esbmc-reference/build/src/esbmc/esbmc" \
    ./scripts/verify_esbmc.sh delayed-views
```

This build provides 64-bit target models. Apply the execution limits below to
builds and solver runs. Record the upstream commit, patch and binary hashes,
compiler/LLVM/solver versions and test logs.
The reference checks require five positive properties and three intentional
failures at their named assertions. They cover live-object alias identity,
write-through behavior, single evaluation, and pointer/array/function references;
they do not qualify all C++ lifetime rules or the entire modified verifier.
Conditional object-reference binding and large aggregate copying in transaction
harnesses require separate qualification; this patch does not close those gaps.
Rerun affected source proofs with this exact tool identity. Existing results
from another binary are not automatically transferred.
The full runner includes `delayed-views`, `delayed-seed`, `delayed-storage` and
`delayed-history`;
these selections run a reference-identity preflight before starting any
profiles, so an unpatched binary fails immediately.
The preflight does not replace the positive/negative qualification above. Keep
tool failures visible rather than omitting profiles from the inventory or
transferring a different binary's result.

### IEEE frame tool qualification

[ieee-frame.patch](../../configs/esbmc/ieee-frame.patch) applies after the
reference-address patch to the same pinned ESBMC source. It corrects two verifier
operations: a bit-cast must load a materialized temporary or reference operand
before its reference annotation is erased; internal floating-point expression
identity must compare representations, consistent with the existing expression
hash. Numeric IEEE equality remains distinct from expression identity: positive
and negative zero compare numerically equal, and NaN compares unequal to itself.

`./tests/test_esbmc.sh --frames` checks both scalar formats, with fourteen positive
properties and six intentional failures at named assertions. The properties cover
arbitrary IEEE field equality, signed-zero literals, non-NaN representation round
trips, reference/value operands with single evaluation, non-NaN sign inversion
and NaN classification. They do not promise NaN-payload preservation or qualify
all bit-cast types and the entire verifier. Keep these tool cases separate from
estimator requirement coverage.

The delayed-owner selections and `all` also run the signed-zero/temporary preflight
in both formats. A failed preflight stops execution; it is not a substitute for
the full positive/negative qualification. A tool change requires new binary and
patch identities and reruns of affected source evidence, including IEEE storage
frames. Earlier evidence is not transferred to the patched binary. Reviewed
source, qualification and affected reruns are required together; passing this
small qualification alone does not establish that all prior proofs remain valid.

### Aggregate initialization tool qualification

[array-filler.patch](../../configs/esbmc/array-filler.patch) applies to the same
pinned frontend. It materializes Clang's omitted array-element initializers
separately and completes each C++ element's initialization before the next.
Zero-filling alone loses nonzero default member values, including the identity
quaternion in an output history record; sharing or delaying evaluation can also
change constructor and initializer side effects. Production initialization must
not be rewritten to accommodate that verifier defect.

`./tests/test_esbmc.sh --initialization` compares the cases with a native C/C++
compiler and checks both scalar formats. It covers default and partial record
arrays, nested and empty arrays, constructor/default-member evaluation order,
explicit scalar initializers and C zero-fill/designated initialization. Named
negative controls must fail their intended assertions. This qualification
concerns those initialization semantics, not the complete frontend or compiler.
Retain its tool/source identities and native/solver results with the accepted
output-owner evidence; a preflight alone does not replace these checks.

Symbolic conditional copies of nested aggregate values remain an isolated tool
limitation in this build. Output reset source-step profiles bind each Boolean
seed/history case separately; both cases and all physical slots are mandatory.
The production conditional is unchanged. Proof oracles avoid that unsupported
aggregate-copy construct. This does not establish general conditional-copy or
constant-evaluation support for unrelated source.

### Output predictor

`delayed-output` covers the fixed 64-slot output owner in binary32/binary64,
with 64-bit words and the default deployment profile. It checks initialization,
configuration, seed, propagation wiring, feedback, reset and complete storage
frames. Numerical inputs and producer outcomes remain arbitrary, including
failure and non-finite values. Prediction requires admitted 3.8--4.2 ms real
intervals; matching main-state success remains a caller premise.

| Obligation | Required evidence |
|---|---|
| Storage and startup | Actual typed insertion/reset, complete constructor/seed/configuration frames, timing and state-validation producers. |
| Prediction | Exact epoch/lag/overwrite guards, held-before-real propagation, full parameter/IMU forwarding, in-place nominal failure frame and publication only after success. |
| Feedback | Exact delayed/start-epoch admission, quaternion operand order and normalization, original IEEE tracking equations, staging before commit and complete feedback publication. |
| Reset | Exact prefix validation, each seed/history transform, finite/normalization failure, commit after all candidates and seed-only publication without synthetic history. |
| Induction | Physical traversal matches the logical ring; successful staging supplies every committed record, failure preserves visible history, and source contracts preserve temporal order and session capacity. |

The runner requires every physical position, both reset seed/history cases and
every frame partition. Behavioral summaries permit arbitrary storage after a
mutating producer; companion frame profiles check all cells under the same
inputs. Actual typed producers supply copying equations. These overapproximations
do not assume numerical success or let a caller modify state outside its contract.

Large feedback/reset methods use
[`output_steps.py`](../../formal/cpp/esbmc/output_steps.py) to extract pinned
source fragments. Review the complete original method alongside the fragments:
local initialization, call receiver and alias identity, loop bounds and cursor
updates, early return, and publication order must agree. All fragments and their
dependencies are required. Member references and explicit boundary parameters
expose the original operations; they do not substitute new numerical formulas.
An unknown source hash fails extraction rather than silently accepting changed
code. Generated fragments and manifests are verification artifacts, not source
replacements to deploy.

[`OutputPredictor.lean`](../../formal/lean/FormalESKF/OutputPredictor.lean) proves
the stage/commit and temporal relations, exact-real feedback identities and
common-reset residual invariance. Its session invariant supplies nonzero ring
length after seeding; ordered full history with at least two records supplies
positive lag. The IEEE interval recurrence separately preserves its finite
positive envelope. Connect these premises to actual configuration, seed, push,
feedback and reset contracts when reviewing the composition.
The count bound and unique physical ranks ensure that staging visits each slot
at most once, so its per-slot candidate model permits independent producer outcomes.

Acceptance also requires the matching normalization/Hamilton/Exp chain,
same-type in-place nominal transaction, IMU mean-rate/timing producers and output
view contracts. Initialization uses the qualified frontend described above.
The source-step correspondence and source-to-Lean relation remain reviewed
boundaries, not automated refinement proofs. External-input frame checks concern
independent input storage; private workspace is not a supported input alias.
Live caller order, deployed backend/libm accuracy, closed-loop stability,
uniform floating-point error and target resource bounds remain separate.

### Delayed observations and context

The observation contracts concern the single-owner `ObservationHistory` and its
actual fixed profile: GNSS 16, magnetic 64, barometer 128 and context 128 slots.
Both scalar formats use the 64-bit word model and qualified reference/IEEE-frame
verifier. Numerical payloads, including inactive storage, remain arbitrary IEEE
values. Copy frames preserve signed zero and NaN classification, not padding or
NaN payload bits. Public inputs/outputs are external to the owner's private
storage. Live caller sequencing, deployed backends and WCET remain separate.

Acceptance composes the following obligations; neither a caller summary nor an
isolated Lean theorem replaces its dependencies.

| Obligation | Source and model boundary |
|---|---|
| Storage | Actual typed push checks every destination, exact metadata, complete input copying and untouched cells. Actual reset clears all slots and retains length. |
| Admission/control | Actual source identity, seed-source guards, timestamp subtraction, raw/arrival/admitted watermarks, generation, next/begin eligibility, binary64 period recurrence, selection and read-only views. Complete owner frames accompany the behavioral equations. |
| Construction/seed/reset | Actual owner construction/reset and seed orchestration check initialization, retained configuration, input preservation, guards and publication. Seed's typed storage contract is a separate dependency. |
| Public enqueue/pop | Actual callers check producer receiver, complete arguments, order, statuses, discard/retirement diagnostics, metadata and input/storage frames. Source changes clear the selected queue even when later admission rejects it. |
| Context | Actual newest-reference identity and snapshot equations support record. The caller retains the real owner alias; it cannot substitute a fresh reference. Lookup steps check every field, original-message age masks and read-only frames; public wrappers check query/sequence guards. |
| Induction | Physical ring stores and retirement refine logical append/drop; reverse search chooses the first nonfuture cell. Temporal/list lemmas preserve bounded metadata, chronology, current source/generation and snapshot provenance through the source contracts. |

Metadata encodings cover every valid oldest/count/write tuple. Caller frame
profiles partition assertions into complete 16-slot chunks; they do not reduce
storage or input values. All chunks, physical destinations and scalar formats are
mandatory. Summaries check their observed receiver frame **before** the call and
preserve arbitrary producer poststates afterward. The complete partition union
supplies the full frame. Actual typed producers supply value equations omitted
from broad summaries; reviewed write footprints justify enclosing-owner locality.
Whole-root havoc follows the verifier rule described above and restores only
producer-established frames. Actual alias identity plus the complete pre-call
frame supplies the original previous context to the pure snapshot producer.

Large ring/context loops use pinned source extraction. Original methods remain
unchanged; added ring entry points contain no data. Search `break`/context
`return` becomes an explicit stop result; retirement and final metadata statements
are copied. Review extraction against the entire source body, including initial
locals, bounds, increments, early exits and empty/null paths. Typed ESBMC steps,
Lean loop induction and that correspondence review are separate evidence; this is
not an automated C++-to-Lean refinement certificate.

The reviewed source-to-model composition requires:

- Construction/reset establish empty rings and disabled clocks. Seed establishes
  positive equal seed/latest/fusion epochs, source watermarks and one context.
- Actual record guards strictly advance the supplied endpoint. Snapshot retains
  an old message or copies only a strictly newer, nonfuture message; typed push
  preserves every field and supplies the newest slot. Record creates no interior
  gap snapshots; the live caller must supply admitted real IMU intervals.
- Begin advances fusion within latest/lag bounds once per latest endpoint; the
  actual IEEE recurrence preserves the period range needed by half-period casts.
- Valid arrivals may advance arrival time on raw-time rejection. Only a newer raw
  timestamp changes source/generation; `generation <= raw` prevents unsigned
  generation wrap. Effective-time subtraction is guarded before use. Admission
  and event construction read the same unchanged mean through the same pure
  half-period getter, so the frozen offset agrees with the effective epoch.
- Queued effective time is positive and beyond the retained admitted watermark.
  Existing active records are bounded by that watermark, so physical append/drop
  preserves strict order. Source changes first supply an empty old list; unchanged
  sources retain the provenance predicate. Rejections retain the specified frames.
- Pop's inspected counter stays below count, removed never exceeds count, and a
  selected result removes at least one record. The physical retained suffix is
  unchanged and contains only future entries; output on a miss is preserved.
- Context search does not consume data. Snapshot provenance places original
  message time at/before its stored epoch, discharging unsigned-age subtraction;
  missing, stale or explicitly unknown records cannot grant permission.

The boundary needs current positive results and named mutation controls for the
same source/tool/configuration. Timeout, OOM, incomplete partitions or missing
producer evidence remain gaps. No numerical estimator success, sensor validity,
whole-callback atomicity or target timing follows from these queue properties.

## Proof execution resources

Run heavy profiles sequentially, without simultaneous builds or full suites.
Before running, check an OS-enforced aggregate memory/wall-time limit and host
headroom. Internal verifier limits and per-process RSS sampling are not a
host-wide cap; measure process and aggregate use separately.

The proposed profile for a 16 GB host is a 6 GiB aggregate cap, swap disabled,
and at least 4 GiB available beyond the cap. This is an execution prerequisite
to establish and check, not enforcement supplied by the verifier script or a
measured worst-case proof cost. Review producer/caller decomposition when a
query exceeds the budget; never narrow the domain or disable unwinding assertions
to obtain a pass. Record the enforced limits and measured costs with the run.

## Running the agent review

The runner requires Python 3 with `jsonschema` supporting Draft 2020-12, plus the
verifier, analysis, sanitizer-build and agent prerequisites checked by the scripts.
The offline regression checks report validation and runner behavior without
executing real verifiers or an agent. Reports missing required tool results must
be regenerated, not relabeled as having passed unrun checks.

```bash
./tests/test_agent_review.sh
./scripts/run_agent_review.sh report.json
```
