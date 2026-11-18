# Documentation

Read the entry relevant to the task, then its linked contracts. Do not load the
whole document tree by default. [AGENTS.md](../AGENTS.md) and [STYLE.md](../STYLE.md)
provide implementation rules; the root [README](../README.md) provides build/replay
commands. Functional work stays in [FEATURE_TODO.md](../FEATURE_TODO.md);
architecture and verification work stays in [ROADMAP.md](../ROADMAP.md).

## Specifications

Use the [specification writing standard and template](spec/TEMPLATE.md) when
authoring or substantially revising a spec. It defines the shared format;
the [specification index](spec/README.md) locates canonical requirement definitions.

| Subject | Authoritative definition |
|---|---|
| Layer ownership | [Architecture boundaries](spec/eskf-software-architecture-and-verification-boundaries.md) |
| Language/runtime restrictions | [C++ subset](spec/requirements/common/cpp-subset.md), [static analysis](spec/cpp-static-analysis-policy.md) |
| Mathematical interfaces | [State/frames](spec/requirements/core/state.md), [linear algebra](spec/requirements/math/linalg/operations.md), [SO(3)/quaternions](spec/requirements/math/quaternion/multiplication.md), [scalar/failure semantics](spec/requirements/common/numerical-profile.md) |
| Behavioral requirements | [Requirement catalogue](spec/verification-requirements.md) — use the relevant ID/section |
| PX4 deployment | [Module architecture](spec/px4-module-architecture.md), [timing profile](spec/px4-timing-profile.md) |
| Verification | [Strategy](spec/formal-verification-strategy.md), [review/traceability criteria](spec/formal-verification-traceability-and-review-criteria.md), [AST review boundary](spec/cpp-function-inventory-and-formal-proof-review-queue.md) |

Specifications own required behavior and claim limits. A design proposal or local
experiment does not silently override them.

## Guides

- [Configuration](guides/configuration.md): core/runtime options, menu ownership and packaged PX4 settings.
- [PX4 operation](../integrations/px4/README.md): installation, module tests and SITL commands.
- [Verification and evidence](guides/verification.md): tool responsibilities, remaining proof boundaries and evidence retention.
- [Magnetic tilt projection](guides/magnetic-tilt-protection.md): derivation and coordinate interpretation.
- [Validation data](guides/validation-data.md): replay/oracle distinctions and acceptance inputs.

## Pending analysis

Systematic analysis remains planned work: F-05.2 covers the local-frame operating
envelope, F-07.1 covers independent validation and the SIH contact issue, V-04
covers FHA/FMEA/FTA, and V-05 covers generated/dense equivalence. Use the backlogs
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
