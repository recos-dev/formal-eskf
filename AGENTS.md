# Agent Instructions

formal-eskf is a portable C++20 ESKF with INS and rotation-only AHRS configurations.
The numerical core is independent of deployment platforms and linalg backends.

## Start with the task

Run `git status --short`; preserve unrelated edits, staged files and ongoing runs.
Read the affected source, callers and tests, then the relevant contracts below.
Load only the task's documents and dependencies, not the entire spec tree.

| Task | Read |
|---|---|
| Behavior or architecture | [Documentation index](docs/README.md), then the affected [requirements](docs/spec/README.md) |
| C++ implementation | [Coding style](docs/guides/coding-style.md), [C++ subset and ArduPilot design baseline](docs/spec/requirements/common/cpp-subset.md), [.clang-format](.clang-format) |
| PX4 integration | [Module architecture](docs/spec/architecture/px4-module.md), [operation and tests](integrations/px4/README.md) |
| Specification changes | [Writing template and approval rules](docs/spec/TEMPLATE.md) |
| Verification | [Evidence policy](docs/spec/verification/assurance.md), [commands, scope and resource limits](docs/guides/verification.md) |
| Build or CI | [README](README.md), [CI workflow](.github/workflows/ci.yml) |
| Task planning or status | [Roadmap](docs/roadmap.md): preserve IDs, completed scope and agreed order; explain new gaps or priority changes |

Specifications own behavior; logs and analysis proposals do not override them.
Verify paper notation/equations against the reference. Surface conflicts between
requirements, references and code before choosing a different model. Task
completion does not approve a Draft requirement or establish verification.

## Make the change

- Preserve behavior, defaults, numerical evaluation order, failure precedence,
  atomicity and aliasing during refactors. Separate intended behavior changes
  from structural cleanup; do not bundle unrelated work.
- Keep the core fixed-size and bounded, without dynamic allocation, exceptions,
  RTTI or OS facilities. Use existing abstractions; keep backend types, platform
  effects and replay conveniences outside shared interfaces and equations.
- Follow ArduPilot's same-layer design baseline through the linked subset.
  Prefer direct numerical/control flow; avoid redundant wrappers and dispatch.
- Reuse existing scripts. Use uppercase script variables and readable functions.
  Do not vendor external platform code without authorization.
- Write documentation in English. Update affected contracts and
  [traceability entries](formal/agent-review/traceability-map.json); preserve IDs,
  approval states and claim boundaries. Link canonical definitions instead of
  copying requirements, mutable proof statistics or development diaries.

## Validate the affected scope

Choose checks for the changed behavior, dependencies and configurations. Use
existing runners and their `--help`; prefer targeted suites. Read the verification
guide's resource limits before heavy proofs. Documentation-only edits need link,
reference and consistency checks, not a full proof run.

- Use `./scripts/format_cpp.sh --check` to inspect formatting; its default edits
  files. Cppcheck only analyzes. ESBMC `--list` only inventories profiles.
- For ESBMC/review tooling changes, run the applicable `./tests/test_esbmc.sh`
  and `./tests/test_agent_review.sh`; their defaults test offline fixtures, not
  real proof execution or an AI audit. Consult the CI workflow for other checks.
- Verify production paths and all required producer/caller dependencies. Never
  weaken requirements/assertions, assume away failures or substitute a rewritten
  implementation just to pass.
- Keep Lean, ESBMC, native/static checks and agent review distinct; no
  machine-checked Lean-to-C++ refinement is claimed. Report coverage by property,
  assumptions and profile; requirement/theorem/harness/profile counts are distinct.
  Missing/stale evidence, timeouts and incomplete unwinding remain gaps.

## Review and hand off

- Review the complete task diff: what changed, why it is correct, whether behavior
  is preserved, and what the checks establish. Report failures, unrun checks and
  remaining gaps; do not extend claims to unverified targets.
- Stage, commit, push or rewrite commit metadata only when explicitly authorized.
  Select exact task files and review the staged diff. Exclude trial notes, logs
  and generated outputs; retain release evidence as the verification guide requires.
