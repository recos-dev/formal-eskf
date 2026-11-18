# Specification Writing Standard and Templates

Use this guide for new or substantially revised specifications in `docs/spec/`.
Choose the requirements or architecture template below. A small component may
combine both in one document, with requirements and design in distinct sections.
Do not create a separate document for every class or function.

The [review criteria](formal-verification-traceability-and-review-criteria.md#record-ownership)
own the requirement-to-evidence process. This guide defines its specification
inputs; it does not add estimator behavior or verification claims.

## Writing rules

- Write in English, with one descriptive H1 title. Use lowercase, hyphenated
  filenames describing the subject, without dates or implementation status.
- Use **shall** and **shall not** for requirements, **may** for permission and
  **should** for recommendations. Separate rationale from obligations. Replace
  vague words such as "safe", "fast" and "robust" with verifiable behavior.
- Give independently reviewable obligations stable IDs. Preserve existing IDs
  and child clauses; file moves or function renames do not justify renumbering.
  Clause, theorem and harness counts are not requirement-coverage counts.
- Keep each requirement definition in one canonical location. Reference existing
  requirements instead of restating them. Distinguish parent requirement IDs
  from supporting sources and rationale; do not invent a parent link.
- Describe behavior before implementation. Prescribe a mechanism only when it
  is a necessary interface, ownership, timing or other design constraint.
- Identify caller obligations explicitly: an assumed input property does not
  imply a callee check. Give independently reviewed assumptions of use stable
  IDs, or link their existing definitions.
- Cite paper versions/equations, upstream revisions/locations and project
  adaptations. A reference alone does not establish equivalent behavior.
- Use document status **Draft** or **Approved**. Approval accepts the specification,
  not implementation or verification completion. Record the reviewed revision and
  approval separately; Git owns revision history. Omit manual development diaries.
- Remove instructions and unused placeholders from copied templates. Omit optional
  sections when unnecessary. Unknown behavior or bounds remain open decisions,
  not invented defaults or "not applicable" entries.

## Requirements template

Use this for mathematical, API, runtime and deployment behavior. Each requirement
identifies its basis, applicability, assumptions, statement and acceptance method.
Shared scope and assumptions may be inherited by reference instead of repeated.

````markdown
# <Subject> Requirements

**Status:** Draft

## Summary

<What behavior does this specification define, and why is it needed?>

## Scope and assumptions

<Identify the owning layer, applicable configurations, modes and targets.>
<State exclusions and link shared conventions or assumptions of use.>
<Identify who establishes each assumption and who checks each input condition.>

## Requirements

### <Stable ID>: <Behavior>

**Basis:** <Parent requirement IDs, if any; supporting source and rationale for
this requirement or project adaptation. Distinguish these relationships.>

**Applies to:** <Scope above, or a narrower layer/configuration/operating domain.>

**Assumptions:** <Applicable shared assumptions or additional caller obligations.>

**Statement:** <Under the specified condition, the named subject shall produce
the required observable response. Include boundaries and state effects.>

**Acceptance / verification:** <Pass/fail criterion and necessary method:
proof, test, analysis or inspection. State the property each method establishes;
a test filename or theorem name alone is not an acceptance criterion.>

## Verification boundary

<State evidence needed for acceptance, dependencies and limitations. Distinguish
exact mathematics, floating-point/source checks, backend conformance and target
validation where applicable. Do not require every method for every requirement.>
<Link registered verification references through the traceability map. Identify
missing evidence explicitly. Keep execution results in identified reports.>

## Open decisions

<Optional: unresolved behavior or bounds, affected IDs and existing backlog links.
Unresolved normative behavior prevents approval of the affected specification.>

## References

<Authoritative project contracts and versioned external sources, with relevant
equations or sections.>
````

Address these concerns where applicable; use tables or equations when clearer
than prose. They need not become separate sections in every specification.

| Concern | Required content |
|---|---|
| Numerical behavior | Domains, dimensions, units, frames, equations, scalar assumptions and numerical acceptance bounds |
| Interfaces and failures | Inputs/outputs, rejection versus failure, error precedence where required, aliases and preserved/modified state |
| Runtime and deployment | State owner, execution order, timestamp meaning, bounded work/storage, allocation/blocking, deadlines and overload behavior |
| Configuration | Applicable modes, parameter units/ranges/defaults and owner; link existing definitions rather than copying them |

For conditional behavior, use a table such as:

| Condition | Required response/status | Modified state | Preserved state |
|---|---|---|---|
| <Condition and precedence, if relevant> | <Observable result> | <Allowed changes> | <Required preservation> |

Keep the table under its owning requirement ID. State whether cases are exhaustive
and how overlapping conditions are resolved. Mathematical specifications may
explicitly delegate API or deployment rules to a named contract.

## Architecture template

Use this to explain how responsibilities, interfaces and execution satisfy
requirements. Reference canonical requirements; any new obligation needs an
explicit requirement definition rather than being hidden in design prose.

````markdown
# <Subject> Architecture

**Status:** Draft

## Summary

<Purpose of the component/layer and the boundary this design establishes.>

## Scope and requirements

<Applicable configurations, assumptions, exclusions and canonical requirement IDs.>

## Responsibilities and interfaces

<Components, owned state/resources, callers, inputs/outputs and dependencies.>
<Use a structural diagram or table where useful. Explain why boundaries exist;
do not mirror every source file or helper function.>

## Execution and state

<Control/data flow, update order, state transitions and failure/recovery behavior.>
<For runtime/module designs: execution context, scheduling, concurrency, clocks,
buffer ownership and overload behavior; reference bounded-work/storage and
allocation/blocking requirements. Distinguish requirements from measured timing.>

## Design decisions

<Consequential choices, reasons and tradeoffs. Explain how they satisfy relevant
requirements, including dependencies and assumptions of use.>

## Verification boundary

<Allocate requirement verification to components and integration boundaries.
State what unit proofs/tests establish and what needs integration or target
evidence. Link the registered mapping; identify gaps without claiming results.>

## Open decisions

<Optional: unresolved design choices, affected requirement IDs and backlog links.>

## References

<Authoritative contracts and versioned external design references.>
````

## Traceability and change handling

Keep definitions in their owning specs, verification references in the
[traceability map](../../formal/agent-review/traceability-map.json), and results
in execution reports. The [requirement catalogue](verification-requirements.md)
identifies applicable requirements and their owning definitions. Do not maintain
duplicate definitions or manually repeated PASS/coverage totals.

Relationships distinguish derivation, design allocation and verification. A link
establishes where to inspect evidence, not that it is adequate or has passed.
These are authoring rules, not a claim that current tooling automatically enforces
all relationships.

A baseline identifies a specific revision of specifications and their mappings.
Keep its approval record and evidence identity distinct. When requirements,
assumptions, configurations or dependencies change, identify affected designs and
verification items; record why evidence remains applicable or needs renewal.
Editorial changes do not themselves require rerunning proofs. Semantic changes
do not inherit an old PASS merely because the requirement ID is unchanged.

When moving definitions, preserve IDs and anchors or update their references.
Register new scope explicitly; unmapped work is not implicitly covered by a nearby
requirement. Link work items to existing backlogs rather than creating another list.

## Reference practices

These sources inform the tailored format; adopting it does not establish
certification compliance or require a particular tool.

- [RTEMS requirements guidance](https://docs.rtems.org/docs/main/eng/req/req-for-req.html):
  identifiable, verifiable requirements and explicit applicability.
- [Eclipse S-CORE requirements concept](https://eclipse-score.github.io/reference_integration/main/process_methods_tools/score_process_description/process_areas/requirements_engineering/requirements_concept.html):
  requirement levels, rationale, relationships and assumptions of use.
- [AUTOSAR OS Requirements, R23-11](https://www.autosar.org/fileadmin/standards/R23-11/CP/AUTOSAR_CP_SRS_OS.pdf),
  sections 3 and 4: requirement quality and structured rationale/dependencies.
- [NASA F' RateGroupDriver SDD](https://github.com/nasa/fprime/blob/43b14c540171f3ee38416dcefe1292d14ed0f118/Svc/RateGroupDriver/docs/sdd.md):
  concise component requirements, design, interfaces and execution constraints.
- [Jama relationship rules](https://help.jamasoftware.com/en/administration/organization-administrator/managing-content/managing-relationships/set-up-relationship-rules.html),
  [baselines](https://help.jamasoftware.com/ah/en/manage-content/baselines.html) and
  [test runs](https://help.jamasoftware.com/ah/en/test/test-runs.html): typed
  relationships and separation of snapshots, verification definitions and results.
