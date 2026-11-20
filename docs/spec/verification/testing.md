# Test Specification and Coverage Standard

**Status:** Draft

**Applies to:** Native, replay, PX4 integration and verification-tool tests.
Requirements define behavior; test specifications define how to check it;
execution records show what actually ran. The [core/runtime test design](tests/README.md)
allocates the current clean-slate scope. Retained in-scope core/runtime tests are qualified against that design; their
existence alone is not coverage. This format does not approve
existing cases or results.

## Test specification template

Keep one file per cohesive test family under `docs/spec/verification/tests/`
when adopted. Related cases share setup and profiles. Reference executable
fixtures and procedures instead of duplicating their implementation; retain the
input conditions, expected behavior and oracle rationale in the specification.
Use case sections instead of table rows when the content needs more space.

````markdown
# <Test family>

**Status:** Draft
**Scope:** <APIs/components, requirement links and exclusions.>
**Level / purpose:** <Unit, component, integration or system / regression, etc.>
**Profiles:** <Applicable configurations/backends/scalars/options/targets, or link.>
**Setup:** <Shared fixtures, clocks, real/substituted dependencies and preparation/
cleanup procedure. Link detailed values, data and tool/environment prerequisites.>
**Regression:** <When this family runs: affected changes, scheduled or release checks.>

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-<requirement-ID>-<suffix> | <Specific clause link> | <Initial state, input partition, action; profile override if needed> | <Status, values and changed/preserved state; reference/derivation and tolerance rationale> | <File, function and selector, or Pending> |

## Gaps and exclusions

<Uncovered clauses/partitions/profiles, justified alternate methods, unresolved
criteria and existing task links. Omit only if none remain.>

## Review

<Pending, or approval reference identifying reviewer, revision and case/profile scope.>
````

The five case columns answer: which test, which obligation, which scenario,
what constitutes correctness, and where it runs. Shared fields apply unless a
case explicitly overrides them. A numeric criterion or procedure may be linked;
its identified version is part of the case. Run-specific PASS text stays outside
this document.

## Shared rules

- **Identity and ownership:** Give each logical case a stable ID and retain it
  through file moves or allocation changes. Do not reuse retired IDs. A case may
  link several requirements. Tool cases use `TC-TOOL-<tool>-<suffix>` and name their
  policy/design owner. `TC-UNALLOCATED-<area>-<suffix>` records a missing owner;
  retain the ID after allocation. The family path and revision identify its document.
- **Levels:** Unit checks one operation/small class; component checks an owner
  across calls; integration checks connected components; system checks a declared
  deployment. Regression is a purpose at any level. Shared setup must identify
  substituted dependencies; host tests do not establish target behavior.
- **Design:** Cover applicable normal/boundary inputs, invalid consumed inputs,
  failure precedence, allowed aliases, preserved state and recovery. Stateful
  cases need event order, clocks and intermediate observations. Include fixture
  identity or a reproducible generation recipe/seed. Unsupported undefined usage
  is not a promised rejection case. Explain exclusions.
- **Oracle:** Use requirements, independent calculations or a reviewed model.
  Shared production helpers only check wiring where they supply both sides;
  dense/generated agreement cannot validate a shared model. Apply
  [C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md) for
  residuals, units, domains and justified tolerances. Exact statuses and preservation
  retain their specified equality semantics. Unresolved bounds remain gaps.
- **Development:** For behavior changes, establish the requirement and a test
  exposing the missing/incorrect behavior before changing production code. Then
  implement, run the affected cases, refactor and rerun regression. Retained defect
  cases link their reproducer and defect record. Do not accept current outputs as
  the oracle or weaken assertions to pass.
- **Traceability and coverage:** Allocate each requirement clause/input partition
  and applicable profile to cases, justified alternate evidence or an explicit
  gap. Family tables own the test allocation; machine catalogs reference and
  reconcile it. Review unallocated code/tests too. Report planned obligations,
  reviewed cases, executed case/profile pairs and accepted requirements separately.
  Parent composition and measured statement/branch coverage need their own review;
  case totals and finite sequences are not proof of completeness or invariance.
- **Selection:** Include affected callers, dependencies, configurations, fixtures
  and stateful sequences in regression. Justify profile reductions and test relevant
  interactions; no automatic Cartesian product is required. Reconcile the whole
  supported scope before release. Standardized cases must be selectable and report
  individual outcomes; reject unknown/empty selections and duplicate case/profile
  registrations. Assertions remain active in Release.
- **Approval:** Follow [TEMPLATE.md](../TEMPLATE.md#requirement-status): Draft and
  Approved refer to reviewed scope/revision, not execution success. Family status
  applies to its cases unless explicitly overridden. Substantive changes return
  affected scope to Draft. Acceptance needs approved applicable requirements and
  test criteria plus current evidence; AI self-review or commit permission is
  not approval. Lean, ESBMC, native/static checks and AI review remain distinct
  under the [assurance policy](assurance.md).

## Execution evidence

The runner should produce one bundle for related cases, with shared metadata
recorded once. These are acceptance requirements for evidence. The
[native case runner](tests/README.md#run-the-tests) implements case/source
reconciliation and local results; profile completeness, dependency identity,
approved numerical criteria and release retention still need explicit review.

| Record | Minimum content |
|---|---|
| Run | Run ID, start/end time with timezone, executor/runner and exact commands; spec/source/dependency/fixture identities including dirty changes; build/tool/target/executable identity, full profiles and relevant floating-point modes; environment/data readiness. |
| Case result | Case ID and profile, outcome, duration, actual observations/assertion evidence and retrievable artifact hashes. Preserve failed/blocked attempts, their reproducer/investigation reference and reruns. |
| Assessment | Expected versus executed cases/profiles, missing/stale evidence, deviations and unresolved risks, with the review decision and baseline. |

Outcomes are `Pass`, `Fail`, `Blocked` or `Not run`. Pass requires every specified
observation to meet its criterion. A demonstrated violation is Fail; an unavailable
environment or runner timeout without a valid verdict is Blocked. An observed
requirement deadline miss is Fail. Expected API
rejection can be Pass. Exclusion and freshness are separate from execution outcome.
Missing, duplicate, unknown, stale or incomplete records cannot yield a complete
PASS; required failures, blocked/not-run cases and unresolved criteria prevent
acceptance. A suite exit code cannot invent per-case evidence. AI review cannot
replace runner outcomes. Hashes identify content, not the trustworthiness of its producer.

Keep logs outside specifications and preserve accepted evidence under the
[retention policy](../../guides/verification.md#record-ownership-and-reuse).
The catalog, report schema and runner integration belong to
[V-02](../../roadmap.md#v-02--design-and-execution-evidence-pilot). Design the
core/runtime scope first and qualify executable bindings against it. Adopting
this format or passing native tests does not establish acceptance of all evidence.

## Basis and tailoring

[ISO/IEC/IEEE 29119-3:2021](https://standards.ieee.org/ieee/360/7499/) supplies the
structural reference. The [public preview](https://webstore.ansi.org/preview-pages/ISO/preview_ISO%2BIEC%2BIEEE%2B29119-3-2021.pdf)
was used for this document-category cross-reference, not a full conformance review.

| Reference area | Project adaptation |
|---|---|
| Identity/policy, §§5–6 | Git identity, approval rules and shared policy above. |
| Planning/design/procedures, §7.2 and §§8.2–8.4 | Inventory plus one family record and case table. |
| Data/environment, §§8.5–8.8 | Shared setup; readiness captured per run. |
| Results/incidents/completion, §§8.9–8.11 and §§7.3–7.4 | Execution bundle and linked findings, outside specs. |

[NASA topic 5.14](https://swehb.nasa.gov/spaces/SWEHBVC/pages/50888777/5.14%2B-%2BTest%2B-%2BSoftware%2BTest%2BProcedures)
informs procedure content;
[RTEMS §§5.6.7–5.6.9](https://docs.rtems.org/docs/main/eng/req/howto.html) informs
configuration/state partitions;
[Jama test runs](https://help.jamasoftware.com/ah/en/test/test-runs.html) informs
separation of definitions from executions. This is a project format, not a claim
of ISO conformance, allocated ASIL/DAL or certification.
