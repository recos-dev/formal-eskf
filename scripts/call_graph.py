#!/usr/bin/env python3
"""Export a source call graph and a stable, manual redundancy-review queue.

Use trusted compilation databases. This is a lexical AST graph, not an exhaustive
execution graph, a proof, or an automatic decision to remove defensive checks.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
import inventory_cpp as AST


class Calls:
    def __init__(SELF):
        SELF.SITES = {}

    def collect(SELF, TU, ROOT, DIRECTORY, SOURCES, PROFILE):
        from clang import cindex

        def target_record(TARGET):
            RECORD = {"target_id": AST.function_identity(TARGET, ROOT, DIRECTORY)[0],
                      "target_name": AST.qualified_name(TARGET), "target_location": None}
            DEFINITION = TARGET.get_definition() or TARGET.canonical
            if DEFINITION.location.file:
                TARGET_FILE = (DIRECTORY / DEFINITION.location.file.name).resolve()
                RECORD["target_location"] = {"file": AST.display_path(ROOT, TARGET_FILE),
                                             "line": DEFINITION.location.line, "column": DEFINITION.location.column}
            return RECORD

        def overload_candidates(CURSOR):
            # Only the callee expression, never calls/references in arguments.
            if CURSOR.kind.name == "OVERLOADED_DECL_REF":
                COUNT = cindex.conf.lib.clang_getNumOverloadedDecls(CURSOR)
                return [target_record(cindex.conf.lib.clang_getOverloadedDecl(CURSOR, I)) for I in range(COUNT)]
            return [TARGET for CHILD in CURSOR.get_children() for TARGET in overload_candidates(CHILD)]

        def visit(CURSOR, OWNER=None, CONTEXT="body"):
            FILE = (DIRECTORY / CURSOR.location.file.name).resolve() if CURSOR.location.file else None
            if FILE is not None and FILE not in SOURCES:
                return
            KIND = CURSOR.kind.name
            if KIND == "LAMBDA_EXPR":
                # Capture initializers execute in the enclosing function; the
                # closure body belongs to its own operator().
                for CHILD in CURSOR.get_children():
                    if CHILD.kind.name != "COMPOUND_STMT":
                        visit(CHILD, OWNER, "capture_initializer")
                for METHOD in CURSOR.type.get_declaration().get_children():
                    if METHOD.spelling == "operator()":
                        visit(METHOD)
                return
            if FILE in SOURCES and KIND in AST.FUNCTION_KINDS:
                OWNER = AST.function_identity(CURSOR, ROOT, DIRECTORY)[0]
            if KIND == "PARM_DECL":
                CONTEXT = "default_argument"
            TARGET = CURSOR.referenced if KIND in ("CALL_EXPR", "DECL_REF_EXPR") else None
            CAPTURE_REFERENCE = (CONTEXT == "capture_initializer" and KIND == "DECL_REF_EXPR" and
                                 TARGET and TARGET.kind.name in AST.FUNCTION_KINDS)
            if FILE in SOURCES and OWNER and (KIND == "CALL_EXPR" or CAPTURE_REFERENCE):
                SITE = {"caller": OWNER, "file": AST.display_path(ROOT, FILE),
                        "line": CURSOR.location.line, "column": CURSOR.location.column,
                        "start": CURSOR.extent.start.offset, "end": CURSOR.extent.end.offset,
                        "context": CONTEXT, "expression": SOURCES[FILE][CURSOR.extent.start.offset:CURSOR.extent.end.offset].decode(errors="replace")[:200],
                        "target_id": None, "target_name": None, "target_location": None,
                        "dispatch": "unresolved", "candidates": []}
                if TARGET and TARGET.kind.name in AST.FUNCTION_KINDS:
                    SITE.update(target_record(TARGET))
                    SITE["dispatch"] = "virtual" if TARGET.kind.name == "CXX_METHOD" and TARGET.is_virtual_method() else "direct"
                    if CAPTURE_REFERENCE:
                        # Some Clang versions flatten capture initializer calls
                        # to references. Preserve a conservative candidate edge.
                        SITE["dispatch"] = "capture_reference"
                elif TARGET and TARGET.kind.name in ("VAR_DECL", "PARM_DECL", "FIELD_DECL"):
                    TYPE = TARGET.type.get_canonical()
                    if TYPE.kind.name in ("LVALUEREFERENCE", "RVALUEREFERENCE"):
                        TYPE = TYPE.get_pointee().get_canonical()
                    if TYPE.kind.name in ("POINTER", "MEMBERPOINTER"):
                        TYPE = TYPE.get_pointee().get_canonical()
                    FUNCTION_POINTER = TYPE.kind.name in ("FUNCTIONPROTO", "FUNCTIONNOPROTO")
                    SITE["dispatch"] = "indirect" if FUNCTION_POINTER else "dependent_callable"
                    SITE["target_name"] = TARGET.spelling
                else:
                    CHILDREN = list(CURSOR.get_children())
                    if CHILDREN:
                        SITE["candidates"] = sorted(overload_candidates(CHILDREN[0]), key=lambda R: R["target_id"])
                    if SITE["candidates"]:
                        SITE["dispatch"] = "overload_candidates"
                KEY = AST.digest(AST.canonical(SITE))
                STORED = SELF.SITES.setdefault(KEY, {"id": KEY, **SITE, "translation_units": []})
                if PROFILE not in STORED["translation_units"]:
                    STORED["translation_units"].append(PROFILE)
            for CHILD in CURSOR.get_children():
                visit(CHILD, OWNER, CONTEXT)

        visit(TU.cursor)

    def resolve(SELF, INVENTORY):
        FUNCTIONS = {F["id"]: F for F in INVENTORY["functions"]}
        LOCATIONS = {}
        for FUNCTION in FUNCTIONS.values():
            for DECL in FUNCTION["declarations"]:
                KEY = (DECL["file"], DECL["line"], DECL["column"])
                LOCATIONS.setdefault(KEY, set()).add(FUNCTION["id"])
        FILES = {F["file"] for F in INVENTORY["files"]}

        def resolve_target(TARGET):
            TARGET["callee"] = TARGET["target_id"] if TARGET["target_id"] in FUNCTIONS else None
            TARGET["resolution"] = "identity" if TARGET["callee"] else "unresolved"
            LOCATION = TARGET["target_location"]
            if TARGET["target_id"] and not TARGET["callee"] and LOCATION:
                CANDIDATES = LOCATIONS.get((LOCATION["file"], LOCATION["line"], LOCATION["column"]), set())
                if len(CANDIDATES) == 1:
                    TARGET["callee"] = next(iter(CANDIDATES))
                    TARGET["resolution"] = "declaration_location"
                else:
                    TARGET["resolution"] = "external" if LOCATION["file"] not in FILES else "unmapped"

        for SITE in SELF.SITES.values():
            SITE["translation_units"].sort()
            resolve_target(SITE)
            for TARGET in SITE["candidates"]:
                resolve_target(TARGET)
        return sorted(SELF.SITES.values(), key=lambda S: (S["caller"], S["file"], S["line"], S["column"], S["id"]))


def git_snapshot(ROOT):
    RESULT = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=ROOT, text=True, capture_output=True, check=False)
    if RESULT.returncode or Path(RESULT.stdout.strip()).resolve() != ROOT:
        return None
    HEAD = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    STATUS = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, text=True)
    return {"head": HEAD, "tracked_changes": STATUS.splitlines()}


def graph_payload(RESULT):
    return {K: V for K, V in RESULT.items() if K != "fingerprint"}


def validate_graph(RESULT):
    if RESULT.get("version") != 1 or RESULT.get("fingerprint") != AST.digest(AST.canonical(graph_payload(RESULT))):
        raise ValueError("invalid call-graph fingerprint/version")
    INVENTORY = RESULT["inventory"]
    if INVENTORY.get("fingerprint") != AST.digest(AST.canonical(graph_payload(INVENTORY))):
        raise ValueError("invalid embedded inventory fingerprint")


def adjacency(RESULT):
    OUTGOING = {F["id"]: set() for F in RESULT["inventory"]["functions"]}
    for SITE in RESULT["calls"]:
        if SITE["callee"]:
            OUTGOING[SITE["caller"]].add(SITE["callee"])
        OUTGOING[SITE["caller"]].update(T["callee"] for T in SITE["candidates"] if T["callee"])
    return OUTGOING


def affected_functions(RESULT, PREVIOUS):
    CURRENT = {F["id"]: F for F in RESULT["inventory"]["functions"]}
    if PREVIOUS is None:
        return {ID: ["no_previous_snapshot"] for ID in CURRENT}, []
    validate_graph(PREVIOUS)
    OLD = {F["id"]: F for F in PREVIOUS["inventory"]["functions"]}
    REASONS = {}

    def mark(ID, WHY):
        REASONS.setdefault(ID, set()).add(WHY)

    for ID in CURRENT.keys() | OLD.keys():
        if ID not in OLD:
            mark(ID, "added")
        elif ID not in CURRENT:
            mark(ID, "removed")
        elif CURRENT[ID] != OLD[ID]:
            mark(ID, "declaration_or_profile_changed")
    for SIDE in (RESULT, PREVIOUS):
        OTHER = PREVIOUS if SIDE is RESULT else RESULT
        INPUTS = {I["file"]: I["sha256"] for I in SIDE["inventory"]["inputs"]}
        OTHER_INPUTS = {I["file"]: I["sha256"] for I in OTHER["inventory"]["inputs"]}
        CHANGED = {F for F in INPUTS.keys() | OTHER_INPUTS.keys() if INPUTS.get(F) != OTHER_INPUTS.get(F)}
        UNITS = {U["id"] for U in SIDE["inventory"]["translation_units"] if CHANGED.intersection(U["dependencies"])}
        for FUNCTION in SIDE["inventory"]["functions"]:
            if any(UNITS.intersection(D["translation_units"]) for D in FUNCTION["declarations"]):
                mark(FUNCTION["id"], "translation_unit_dependency_changed")
    GLOBAL_CHANGE = (RESULT["tool_sha256"] != PREVIOUS["tool_sha256"] or
                     RESULT["inventory"]["tool"] != PREVIOUS["inventory"]["tool"] or
                     RESULT["inventory"]["scope"] != PREVIOUS["inventory"]["scope"] or
                     RESULT["inventory"]["translation_units"] != PREVIOUS["inventory"]["translation_units"] or
                     RESULT["config_sha256"] != PREVIOUS["config_sha256"])
    if GLOBAL_CHANGE:
        for ID in CURRENT:
            mark(ID, "tool_configuration_or_profile_changed")
    SITES = lambda DATA, ID: [S for S in DATA["calls"] if S["caller"] == ID]
    for ID in CURRENT.keys() & OLD.keys():
        if SITES(RESULT, ID) != SITES(PREVIOUS, ID):
            mark(ID, "calls_changed")
    REVERSE = {}
    for SIDE in (RESULT, PREVIOUS):
        for CALLER, CALLEES in adjacency(SIDE).items():
            for CALLEE in CALLEES:
                REVERSE.setdefault(CALLEE, set()).add(CALLER)
    PENDING = list(REASONS)
    SEEN = set(PENDING)
    while PENDING:
        for CALLER in REVERSE.get(PENDING.pop(), set()):
            mark(CALLER, "reachable_callee_changed")
            if CALLER not in SEEN:
                SEEN.add(CALLER)
                PENDING.append(CALLER)
    return {ID: sorted(REASONS.get(ID, [])) for ID in CURRENT}, sorted(OLD.keys() - CURRENT.keys())


def review_groups(RESULT, ENTRIES):
    FUNCTIONS = RESULT["inventory"]["functions"]
    OUTGOING = adjacency(RESULT)
    ROOTS = set()
    for ENTRY in ENTRIES:
        MATCHES = {F["id"] for F in FUNCTIONS if ENTRY in (F["id"], F["qualified_name"])}
        if not MATCHES:
            raise ValueError(f"entry not found in supplied profiles: {ENTRY}")
        ROOTS.update(MATCHES)
    GROUPS = []
    for ROOT in sorted(ROOTS):
        REACHABLE, PENDING = set(), [ROOT]
        while PENDING:
            ID = PENDING.pop()
            if ID not in REACHABLE:
                REACHABLE.add(ID)
                PENDING.extend(OUTGOING[ID] - REACHABLE)
        GROUPS.append({"entry": ROOT, "functions": sorted(REACHABLE),
                       "open_calls": [S["id"] for S in RESULT["calls"]
                                      if S["caller"] in REACHABLE and (not S["callee"] or S["dispatch"] != "direct")]})
    ASSIGNED = {ID for GROUP in GROUPS for ID in GROUP["functions"]}
    return GROUPS, sorted(OUTGOING.keys() - ASSIGNED)


def build_graph(ROOT, CONFIG, DATABASES, ENTRIES, PREVIOUS=None):
    TOOLS = {PATH: AST.digest(PATH.read_bytes()) for PATH in (Path(__file__), Path(AST.__file__))}
    COLLECTOR = Calls()
    INVENTORY = AST.inventory(ROOT, CONFIG, DATABASES, COLLECTOR.collect)
    if any(AST.digest(PATH.read_bytes()) != HASH for PATH, HASH in TOOLS.items()):
        raise ValueError("call-graph tool changed during extraction")
    RESULT = {"version": 1, "inventory": INVENTORY, "calls": COLLECTOR.resolve(INVENTORY),
              "git": git_snapshot(ROOT), "config_sha256": AST.digest(CONFIG.read_bytes()),
              "tool_sha256": {PATH.name: HASH for PATH, HASH in TOOLS.items()},
              "limitations": [
                  "Potential source calls in supplied profiles; not an exhaustive runtime or template-instantiation graph.",
                  "Virtual calls name declared targets, not all runtime overrides. Indirect/dependent calls can remain unresolved.",
                  "Implicit/defaulted operations and compiler-generated calls need manual review; default-argument expressions are labeled.",
                  "Global initializers outside functions are not roots in this function review graph.",
                  "Function references in lambda capture initializers are conservative candidate edges, not confirmed calls.",
                  "Dependent overload candidates come from Clang lookup; group reachability/impact includes all candidates, without claiming final overload selection or exhaustive ADL.",
                  "Declaration-location links group specializations for review; exact target identities remain in call sites.",
                  "Unchanged inputs do not mean reviewed or proved. No defensive check is removed automatically.",
                  "Impact propagates only along known edges; unresolved calls require manual impact review."]}
    RESULT["groups"], RESULT["unassigned_functions"] = review_groups(RESULT, ENTRIES)
    REASONS, RESULT["removed_functions"] = affected_functions(RESULT, PREVIOUS)
    OUTGOING = adjacency(RESULT)
    RESULT["review_queue"] = [
        {"review_id": AST.digest(ID.encode()), "function": ID,
         "callers": sorted(CALLER for CALLER, CALLEES in OUTGOING.items() if ID in CALLEES),
         "callees": sorted(OUTGOING[ID]), "status": "unreviewed",
         "change_reasons": REASONS[ID], "requires_recheck": bool(REASONS[ID])}
        for ID in sorted(OUTGOING)]
    RESULT["fingerprint"] = AST.digest(AST.canonical(RESULT))
    return RESULT


def markdown(RESULT):
    FUNCTIONS = {F["id"]: F for F in RESULT["inventory"]["functions"]}
    NODE = {ID: f"n{INDEX}" for INDEX, ID in enumerate(sorted(FUNCTIONS))}
    TEXT = ["# Call graph review inventory", "", f"Snapshot: `{RESULT['fingerprint']}`", "",
            f"{len(FUNCTIONS)} functions; {len(RESULT['calls'])} call sites; {len(RESULT['groups'])} review groups; "
            f"{len(RESULT['unassigned_functions'])} functions outside selected groups.", ""]
    TEXT.extend(f"- {LINE}" for LINE in RESULT["limitations"])
    TEXT.extend(["", "For each function, inspect input guarantees, intervening mutations/aliases, failure precedence, "
                 "and unchanged-output obligations before deciding whether repeated validation or computation is redundant. "
                 "Record the stable review ID and this snapshot with the decision. All rows start unreviewed.", ""])
    for INDEX, GROUP in enumerate(RESULT["groups"], 1):
        TEXT.extend([f"## {INDEX}. {FUNCTIONS[GROUP['entry']]['qualified_name']}", "",
                     f"Entry: `{GROUP['entry']}`. Open/external call sites: {len(GROUP['open_calls'])}.", "", "```mermaid", "flowchart TD"])
        for ID in GROUP["functions"]:
            FUNCTION = FUNCTIONS[ID]
            DECL = next((D for D in FUNCTION["declarations"] if D["definition"]), FUNCTION["declarations"][0])
            LABEL = FUNCTION["qualified_name"].replace('"', "&quot;").replace("<", "&lt;").replace(">", "&gt;")
            LABEL += f"<br/>{Path(DECL['file']).name}:{DECL['line']}"
            TEXT.append(f'    {NODE[ID]}["{LABEL}"]')
        EDGES = set()
        OPEN = {}
        for SITE in RESULT["calls"]:
            if SITE["caller"] not in GROUP["functions"]:
                continue
            DEST = NODE.get(SITE["callee"])
            for TARGET in SITE["candidates"]:
                if TARGET["callee"]:
                    EDGES.add(f'    {NODE[SITE["caller"]]} -. candidate .-> {NODE[TARGET["callee"]]}')
            if not SITE["callee"]:
                KEY = (SITE["caller"], SITE["dispatch"], SITE["resolution"], SITE["target_name"])
                DEST = "x" + AST.digest(AST.canonical(KEY))[:16]
                GROUPED = OPEN.setdefault(DEST, {"site": SITE, "count": 0})
                GROUPED["count"] += 1
            ARROW = "-->" if SITE["dispatch"] == "direct" and SITE["callee"] else "-.->"
            EDGES.add(f'    {NODE[SITE["caller"]]} {ARROW} {DEST}')
        for DEST, GROUPED in sorted(OPEN.items()):
            SITE = GROUPED["site"]
            LABEL = (SITE["target_name"] or SITE["dispatch"]).replace('"', "&quot;").replace("<", "&lt;").replace(">", "&gt;").replace("\n", " ")
            TEXT.append(f'    {DEST}["{LABEL}: {GROUPED["count"]} open call site(s)"]')
        TEXT.extend(sorted(EDGES))
        TEXT.extend(["```", ""])
    TEXT.extend(["## Function queue", "", "| # | Review ID | Function / signature | Source | Recheck reasons |", "|---|---|---|---|---|"])
    for INDEX, ROW in enumerate(RESULT["review_queue"], 1):
        FUNCTION = FUNCTIONS[ROW["function"]]
        DECL = next((D for D in FUNCTION["declarations"] if D["definition"]), FUNCTION["declarations"][0])
        NAME = FUNCTION["qualified_name"].replace("|", "&#124;")
        SIGNATURE = "; ".join(FUNCTION["signatures"]).replace("|", "&#124;")
        REASONS = ", ".join(ROW["change_reasons"]) or "unchanged; still unreviewed"
        TEXT.append(f"| {INDEX} | `{ROW['review_id']}` | `{NAME}` / `{SIGNATURE}` | `{DECL['file']}:{DECL['line']}` | {REASONS} |")
    TEXT.extend(["", "## Call sites", "", "| Caller | Target | Source | Dispatch / resolution |", "|---|---|---|---|"])
    for SITE in RESULT["calls"]:
        CALLER = FUNCTIONS[SITE["caller"]]["qualified_name"].replace("|", "&#124;")
        TARGET = (SITE["target_name"] or SITE["expression"] or "unresolved").replace("|", "&#124;")
        if SITE["candidates"]:
            TARGET += " candidates: " + ", ".join(T["target_name"].replace("|", "&#124;") for T in SITE["candidates"])
        TARGET = TARGET.replace("\n", " ").replace("`", "&#96;")
        TEXT.append(f"| `{CALLER}` | `{TARGET}` | `{SITE['file']}:{SITE['line']}:{SITE['column']}` | "
                    f"{SITE['dispatch']} / {SITE['resolution']} / {SITE['context']} |")
    return "\n".join(TEXT) + "\n"


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    PARSER.add_argument("--config", default="configs/ast-inventory.json")
    PARSER.add_argument("--compile-commands", action="append", type=Path)
    PARSER.add_argument("--entry", action="append", default=[], help="qualified name or inventory function ID; repeat for review groups")
    PARSER.add_argument("--previous", type=Path, help="previous graph for conservative change impact")
    PARSER.add_argument("--output", type=Path, help="JSON graph; also writes a sibling Markdown inventory")
    PARSER.add_argument("--check", type=Path, help="validate graph and source/tool freshness")
    ARGS = PARSER.parse_args()
    try:
        ROOT = ARGS.repo.resolve()
        if ARGS.check:
            if ARGS.compile_commands or ARGS.output or ARGS.previous or ARGS.entry:
                raise ValueError("--check cannot be combined with generation options")
            RESULT = json.loads(ARGS.check.read_text())
            validate_graph(RESULT)
            AST.validate_snapshot(ROOT, RESULT["inventory"])
            for PATH in (Path(__file__), Path(AST.__file__)):
                if RESULT["tool_sha256"].get(PATH.name) != AST.digest(PATH.read_bytes()):
                    raise ValueError(f"stale graph tool: {PATH.name}")
            print("Call graph inputs: unchanged (review status is separate)")
            return 0
        if not ARGS.compile_commands or not ARGS.output or ARGS.output.suffix != ".json":
            raise ValueError("generation requires --compile-commands and --output ending in .json")
        PREVIOUS = json.loads(ARGS.previous.read_text()) if ARGS.previous else None
        RESULT = build_graph(ROOT, AST.repository_path(ROOT, ARGS.config), ARGS.compile_commands, ARGS.entry, PREVIOUS)
        OUTPUTS = [ARGS.output.resolve(), ARGS.output.with_suffix(".md").resolve()]
        INPUTS = {(ROOT / I["file"]).resolve() for I in RESULT["inventory"]["inputs"]} | {Path(__file__).resolve()}
        if ARGS.previous:
            INPUTS.add(ARGS.previous.resolve())
        if INPUTS.intersection(OUTPUTS):
            raise ValueError("output must not overwrite a graph input/previous snapshot")
        # Compute both outputs before publishing. JSON is the authoritative snapshot.
        MARKDOWN = markdown(RESULT)
        AST.write_output(ARGS.output, RESULT)
        ARGS.output.with_suffix(".md").write_text(MARKDOWN)
        print(f"Call graph: {len(RESULT['review_queue'])} functions, {len(RESULT['calls'])} call sites.\nOutput: {ARGS.output}")
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
