# Call-graph review

Use `scripts/call_graph.py` to review repeated validation and computation along
source call chains. It reuses the Clang declaration inventory and trusted CMake
compilation databases. It produces a JSON snapshot and a sibling Markdown file
with group diagrams, a numbered function queue and call-site locations.

## Generate a review snapshot

Install Clang and its Python bindings, then configure the build profiles to
inspect. Configuration does not require compiling executables.

```bash
sudo apt install clang-18 python3-clang-18
cmake -S . -B build/ast -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DESKF_CODEGEN=ON
mkdir -p output/call-graph
python3 scripts/call_graph.py \
    --compile-commands build/ast/compile_commands.json \
    --entry formal_eskf::predict_nominal \
    --entry formal_eskf::correct \
    --output output/call-graph/current.json
```

Repeat `--compile-commands` for other build configurations. Repeat `--entry` for
other qualified names or exact inventory function IDs. A name matching several
overloads creates separate groups. Unknown entries and erroneous ASTs fail;
functions outside the selected groups and unparsed source files remain visible.
Without entries, the function queue still covers the extracted scope but has no
group diagrams.

The default [scope](../../configs/ast-inventory.json) is `include/formal_eskf`.
To review deployment owners too, supply `--config` with an explicit source-root
manifest and compilation databases that actually include those sources. Headers
outside the selected roots are dependency hashes, not reviewed project functions.
The recorded macros describe the compiler command's definitions; header defaults
remain part of the hashed source. Explicit build definitions make profiles easier
to compare.

## Review one group at a time

For each function, inspect its callers, callees and cited source locations:

1. Identify who establishes each precondition and on which paths.
2. Check whether values, aliases or configuration can change before the repeated
   predicate or computation.
3. Check failure precedence, output preservation and successful publication.
4. Record a decision against the full stable `review_id` and snapshot fingerprint.
   A repeated expression alone does not justify removing it.

Solid diagram edges represent resolved direct calls. Dashed edges represent
external/unresolved calls or conservative candidates. Dependent overload
candidates come from Clang lookup, not name guessing; they are included in group
reachability and change impact without claiming a final overload selection.
Virtual dispatch, function pointers, dependent callable objects, lambda capture
references and implicit operations still need manual inspection. This is a
source-review aid, not an exhaustive runtime call graph or a proof.
Diagrams group open call sites by caller, dispatch kind and target name, showing
their counts. Every individual source location remains in the JSON and call-site
table; grouping does not discard unresolved obligations.

The queue starts **unreviewed** every time. It does not import approvals, map
functions to requirements or change the agent-review runner. That separate
baseline/completeness work belongs to [V-03](../roadmap.md#v-03--ast-mapping-and-review-completeness).

## Detect stale review inputs

Keep the previous snapshot under a different name and generate a new one:

```bash
mv output/call-graph/current.json output/call-graph/previous.json
python3 scripts/call_graph.py \
    --compile-commands build/ast/compile_commands.json \
    --entry formal_eskf::predict_nominal --entry formal_eskf::correct \
    --previous output/call-graph/previous.json \
    --output output/call-graph/current.json
python3 scripts/call_graph.py --check output/call-graph/current.json
```

Function IDs follow Clang identities, including file-local identity. Changed
signatures can create removed/added IDs. Declaration/call changes invalidate the
function and known transitive callers, using both old and new edges. Dependency
changes invalidate the affected translation units; tool, scope and profile
changes conservatively invalidate the current queue. Unresolved edges require
manual impact review. Source hashes detect dirty-tree changes; a Git revision
alone cannot identify the inspected code. An unchanged snapshot means only that
these inputs are unchanged, never that a review or proof passed.

Validate the extractor with real AST fixtures:

```bash
python3 tests/tools/test_cpp_inventory.py
python3 tests/tools/test_call_graph.py
```

Generated graphs and local review records belong under `output/`, not in commits.
Accepted design changes belong in the relevant requirements and traceability;
this tool does not create a second proof registry.
