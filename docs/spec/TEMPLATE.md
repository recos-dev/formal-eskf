# Specification Writing Standard and Templates

Use one canonical file per independently reviewable requirement. Group files by
owning layer and functional area under `requirements/`; directory nesting does
not establish a parent relationship. [README.md](README.md) is the spec entry.
Architecture explains design, profiles bind deployment values, and verification
policy defines evidence acceptance. None is a second copy of a requirement.

## Writing rules

- Write in English with one descriptive H1 title. Requirement titles include their
  stable ID. Use lowercase, hyphenated filenames independent of implementation names.
- Use **shall** and **shall not** for new obligations, **may** for permission and
  **should** for recommendations. Existing mandatory wording retains its meaning
  during editorial migration. Separate rationale from acceptance criteria.
- Split distinct models, public operations or caller bindings when they need
  independent scope and acceptance. Keep one operation's conditions, error
  precedence, equations and state effects together. Do not split by scalar axis,
  theorem, source function, macro combination or individual error status.
- Preserve existing IDs, including named child clauses. A child clause need not
  have its own file. A parent owns composition/common behavior and links children;
  it does not copy their equations. Parent acceptance requires all applicable
  child obligations, not merely the presence of links.
- Distinguish **Parent** (derivation) from **Uses** (contract dependency). A caller
  is not automatically a child of its callee. Sources and rationale do not invent
  missing system-level parents, safety allocations or certification claims.
- Each file identifies applicability, assumptions, required behavior, basis and
  acceptance. Use shared-contract links rather than repeating definitions. Name
  who establishes each premise and which checks the operation performs.
- Replace vague claims such as "safe", "fast" and "robust" with observable
  properties. Unknown bounds or undecided behavior remain explicit open decisions.
- Keep notation, domain and failure definitions close to their equations. After
  extraction, replace context-dependent phrases such as "above" with a definition
  or specific link. Preserve relevant tables and diagrams.
- Cite paper versions/equations and upstream revisions/locations. State adaptations
  explicitly. A reference alone does not establish behavioral equivalence.
- Mark proposed behavior **Draft**. Adopted text is identified by its reviewed Git
  baseline; a file-level approval label is not required and is not evidence of
  implementation or proof completion. Do not infer approval from a passing test.
- Remove unused placeholders and optional sections. Avoid repeating a summary,
  scope and statement that say the same thing. Git owns revision history; keep
  development diaries, migration ledgers and execution logs outside specifications.

## Requirement template

The metadata and sections may be shortened when the necessary information is
already explicit. Acceptance must remain identifiable; equations and condition
rows can be acceptance criteria without repeating them in another checklist.

````markdown
# <ID>: <Required behavior>

**ID:** `<Stable ID>`

**Applies to:** <Owning layer, configurations and domain; exclusions where relevant.>

**Parent:** <Optional: linked parent requirement and inherited obligations.>

**Uses:** <Optional: linked shared contracts and dependencies.>

## Basis

<Source and rationale, including project adaptations. A linked parent may own
shared sources. Use Status: Draft when the behavior is still a proposal.>

## Required behavior

<Under the specified condition, the named subject shall produce the observable
response. Define inputs/outputs, assumptions, units/frames, boundaries and state
effects. Distinguish caller premises from checks performed by the operation.>

<Use equations or a condition -> response -> modified/preserved state table when
helpful. State whether cases are exhaustive and resolve overlapping conditions.>

## Acceptance and verification

<Pass/fail properties and necessary methods: proof, test, analysis or inspection.
State each method's domain/configuration, dependency obligations and limits.
Separate exact mathematics, source/IEEE checks, backend and target evidence.
A filename or theorem name alone is not an acceptance criterion.>

## Open decisions

<Optional: unresolved behavior or bounds, affected clauses and existing backlog
links. Missing evidence and undecided behavior are different issues.>

## References

<Optional when sources are already linked in Basis or the relevant clauses.>
````

## Architecture template

Architecture allocates requirements to responsibilities and explains the design.
A new behavioral obligation needs a requirement owner; do not hide it in prose.
A profile records values, applicability and the requirements it instantiates.

````markdown
# <Subject> Architecture

## Scope and requirements

<Purpose, applicable configurations, assumptions, exclusions and requirement IDs.>

## Responsibilities and interfaces

<Owned state/resources, callers, inputs/outputs and dependencies. Use a structural
diagram or table when useful. Explain boundaries without mirroring every file.>

## Execution and state

<Control/data flow, update order, state transitions and failure/recovery.>
<For runtime/module designs: execution context, scheduling, concurrency, clocks,
buffer ownership, overload and allocation/blocking constraints. Reference the
owning requirements; distinguish timing obligations from measured results.>

## Design decisions

<Consequential choices, reasons, tradeoffs and requirement allocation.>

## Verification boundary

<Component versus integration/target obligations, dependencies and exclusions.
Reference requirements and the registered mapping without claiming executions.>

## Open decisions

<Optional: unresolved choices, affected requirement IDs and existing backlog links.>
````

## Traceability and change handling

The [review criteria](formal-verification-traceability-and-review-criteria.md#record-ownership)
own the evidence process. Requirement files own normative text and metadata;
the [traceability map](../../formal/agent-review/traceability-map.json) owns
source/proof/test references; identified reports own execution results. The entry
indexes relationships without repeating definitions or manual PASS totals.

A baseline identifies particular specification and mapping revisions. Keep review
acceptance and execution identity distinct. Changes to obligations, assumptions,
configurations or dependencies require an evidence-impact review. Editorial moves
preserve obligations; they do not turn historical results into new executions.
Missing or stale evidence remains a gap.

Preserve IDs/anchors or update all affected references, including literal symbols
in the map and tooling paths. Register new scope explicitly. A new child file is
not automatically a new passing map row; preserve coverage denominators unless a
separate reviewed scope change adopts different granularity.

## Reference practices

These sources inform the tailored format, not a claim of certification compliance.

- [RTEMS requirements](https://docs.rtems.org/docs/main/eng/req/req-for-req.html):
  identifiable, verifiable behavior and applicability.
- [S-CORE requirements concept](https://eclipse-score.github.io/reference_integration/main/process_methods_tools/score_process_description/process_areas/requirements_engineering/requirements_concept.html):
  derivation, rationale and assumptions of use.
- [AUTOSAR OS Requirements, R23-11](https://www.autosar.org/fileadmin/standards/R23-11/CP/AUTOSAR_CP_SRS_OS.pdf),
  sections 3 and 4: requirement quality and structured rationale/dependencies.
- [NASA F' component design](https://github.com/nasa/fprime/blob/43b14c540171f3ee38416dcefe1292d14ed0f118/Svc/RateGroupDriver/docs/sdd.md):
  concise requirements, interfaces and execution constraints.
- [Jama relationships](https://help.jamasoftware.com/en/administration/organization-administrator/managing-content/managing-relationships/set-up-relationship-rules.html),
  [baselines](https://help.jamasoftware.com/ah/en/manage-content/baselines.html) and
  [test runs](https://help.jamasoftware.com/ah/en/test/test-runs.html): separate
  relationships, version snapshots, verification definitions and results.
