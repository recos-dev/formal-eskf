# Agent Guidelines

## Project

formal-eskf is a portable C++20 error-state Kalman filter with INS and rotation-only AHRS configurations. Keep the numerical core independent of deployment platforms and linear-algebra backends.

## Specifications

Use [docs/README.md](docs/README.md) to locate the relevant specification before
changing behavior; read only the affected contracts and their dependencies.
`docs/spec/` owns requirements and equations. `FEATURE_TODO.md` owns functional
work; `ROADMAP.md` owns architecture and verification work. Neither development
logs nor analysis proposals override an adopted specification.

Use the [specification writing standard and template](docs/spec/TEMPLATE.md) for
new or substantially revised specs. Preserve canonical requirement definitions,
stable IDs and affected traceability references when changing document structure.

Verify paper symbols and equations against the actual reference before citing or implementing them. Report conflicts between specifications, papers and code instead of silently choosing a different model.

## Implementation

- Use fixed-size types and bounded operations. Keep dynamic allocation, exceptions, RTTI and OS facilities outside the shared core.
- Use the existing abstractions; backend types and platform effects must not enter shared-core interfaces or equations.
- Preserve specified behavior, defaults and failure/aliasing contracts unless a change is explicitly in scope.
- Follow [STYLE.md](STYLE.md), the [C++ design baseline and subset](docs/spec/cpp-portability-and-constraints.md#design-baseline) and `.clang-format`. Use ArduPilot's actual same-layer practice as the estimator-design reference while retaining C++20; lack of an upstream prohibition does not justify additional generic machinery. Apply these rules within the task scope; do not start an unsolicited rewrite of existing code.
- Use uppercase script variables and split scripts into readable functions. Avoid unnecessary wrappers, helpers and new scripts.
- Keep platform integration and replay conveniences outside the core. Do not vendor external platform code without approval.

## Verification

- Keep Lean model proofs, bounded ESBMC checks, native tests, static analysis and agent review distinct. They do not constitute a machine-checked Lean-to-C++ refinement.
- Scope claims to the analyzed production paths, assumptions and configurations. Record outstanding abstraction dependencies; do not extend claims to unverified targets.
- Do not weaken requirements or assertions, assume away failures or replace production operations merely to obtain a passing result.
- Maintain affected entries in `formal/agent-review/traceability-map.json`. Distinguish requirements, theorems, harnesses and execution profiles when reporting coverage; do not present their counts as interchangeable.
- Missing or stale evidence, timeouts and incomplete unwinding are gaps, not passes.

Use `README.md` for build setup and `.github/workflows/ci.yml` for CI checks. Reuse the existing `scripts/` entry points and consult their `--help` for options. Run checks appropriate to the change; prefer targeted proof suites over rerunning every proof for every edit.

For verification-tool changes, run `tests/test_esbmc.sh` and/or `tests/test_agent_review.sh` as applicable. Their default modes are offline regression tests, not proof execution or an AI audit.

Use `scripts/format_cpp.sh --check` for read-only formatting checks; its default mode edits files. Cppcheck checks without fixing source. ESBMC `--list` inventories profiles without proving them.

## Collaboration

- Use `FEATURE_TODO.md` as the functional backlog: preserve task IDs, completed scope and agreed order when reporting next steps. Keep architecture/formal plans in `ROADMAP.md`; explain new gaps or proposed priority changes instead of silently regenerating either list.
- Inspect the worktree first; preserve unrelated edits and staged changes. Do not interfere with other ongoing work or verification runs.
- Keep changes scoped to the request. Separate proof, production-code, tooling and documentation changes when that helps review; do not bundle unrelated cleanup.
- Self-review changes before handing them back. Report the result, checks performed and remaining gaps without claiming unrun checks passed.
- Stage, commit, push or rewrite commit metadata only when explicitly requested. When staging, select exact task-related files and review the staged diff; exclude temporary notes and generated outputs.
- Keep documentation concise and avoid duplicating specifications or hardcoding changing proof statistics here.
