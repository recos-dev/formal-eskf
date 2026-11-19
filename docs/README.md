# Documentation

Read the entry relevant to the task, then its linked contracts. Do not load the
whole document tree by default. [AGENTS.md](../AGENTS.md) and the [coding style](guides/coding-style.md)
provide implementation rules; the root [README](../README.md) provides build/replay
commands. The [roadmap](roadmap.md) owns functional, architecture, verification
and tooling work, with stable task IDs and separate acceptance boundaries.

## Specifications

Use the [specification writing standard and template](spec/TEMPLATE.md) when
authoring or substantially revising a spec. It defines the shared format;
the [specification index](spec/README.md) locates canonical requirement definitions.

| Subject | Entry |
|---|---|
| Requirements | [Capability hierarchy and canonical layer index](spec/README.md) |
| Architecture | [Software boundaries](spec/architecture/overview.md), [numerical abstraction](spec/architecture/numerics.md), [PX4 module](spec/architecture/px4-module.md) |
| Deployment profile | [PX4 SIH 250 Hz](spec/profiles/px4-sih-250hz.md) |
| Verification policy | [Claims and traceability](spec/verification/assurance.md), [source inventory](spec/verification/source-inventory.md), [static analysis](spec/verification/static-analysis.md) |

Specifications own required behavior and claim limits. A design proposal or local
experiment does not silently override them.

## Guides

- [Coding style](guides/coding-style.md): naming, readability and organization; language permissions remain in the C++ subset.
- [Configuration](guides/configuration.md): core/runtime options, menu ownership and packaged PX4 settings.
- [PX4 operation](../integrations/px4/README.md): installation, module tests and SITL commands.
- [Verification and evidence](guides/verification.md): tool responsibilities, remaining proof boundaries and evidence retention.
- [Call-graph review](guides/call-graph.md): caller/callee groups, stable function identities and change-impact checks for manual review.
- [Magnetic tilt projection](guides/magnetic-tilt-protection.md): derivation and coordinate interpretation.
- [Validation data](guides/validation-data.md): replay/oracle distinctions and acceptance inputs.

## Pending analysis

Systematic analysis remains planned work: F-05.2 covers the local-frame operating
envelope, F-07.1 covers independent validation and the SIH contact issue, V-04
covers FHA/FMEA/FTA, and V-05 covers generated/dense equivalence. Use the roadmap
linked above for scope and status. Draft analyses require separate review before
being included as a repository baseline; they do not replace requirements.

## Evidence and maintenance

Requirements, source references and evidence identity support future assurance
work; they do not claim ISO certification. The [traceability map](../formal/agent-review/traceability-map.json)
is the source/proof index, not a live pass report. Keep development logs and
trial notes in ignored local output. Retain necessary release evidence as
retrievable artifacts with source/configuration/tool identity; do not duplicate
logs, mutable proof totals or completed-task diaries in this tree.

When code changes, update its authoritative contract/guide and affected mappings.
When only document organization changes, preserve requirement IDs and claim
boundaries, check links and source references, and do not relabel old executions
as fresh verification.
