#!/usr/bin/env python3
"""Exercise call-graph extraction and review invalidation with real Clang ASTs."""

import contextlib
import io
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import call_graph as GRAPH
from test_cpp_inventory import InventoryTests


class CallGraphTests(unittest.TestCase):
    write_database = InventoryTests.write_database

    def setUp(SELF):
        InventoryTests.setUp(SELF)
        SELF.write_database(VARIANTS=(0, 1), SOURCES=("one.cpp",))
        SELF.HEADER = SELF.ROOT / "include/api.hpp"
        SELF.HEADER.write_text(SELF.HEADER.read_text() + '''
namespace example {
struct Base { virtual int act() { return 1; } };
inline int leaf(int x) { return x + 1; }
inline int middle(int x) { return leaf(x); }
inline int root(int x) { return middle(x) + overload(x) + overload(1.0) + identity(x) + external_function(); }
inline int indirect(int (*fn)(int), int x) { return fn(x); }
inline int indirect_ref(int (*&fn)(int), int x) { return fn(x); }
inline int dynamic(Base &b) { return b.act(); }
inline auto lambda_caller() { return [] { return leaf(1); }; }
inline auto lambda_capture() { return [x = leaf(2)] { return x; }; }
template<class T> int dependent(T x) { return x.method(); }
template<class T> T dependent_known(T x) { return identity(x); }
inline int variant() {
#if CONFIG_VARIANT
return selected_one();
#else
return selected_zero();
#endif
}
}
''')

    def graph(SELF, ENTRIES=("example::root",), PREVIOUS=None):
        with contextlib.redirect_stderr(io.StringIO()):
            return GRAPH.build_graph(SELF.ROOT, SELF.CONFIG, [SELF.DATABASE], ENTRIES, PREVIOUS)

    def named(SELF, RESULT, NAME):
        return next(F for F in RESULT["inventory"]["functions"] if F["qualified_name"] == NAME)

    def test_direct_overloads_templates_external_and_reachability(SELF):
        RESULT = SELF.graph()
        ROOT = SELF.named(RESULT, "example::root")["id"]
        CALLS = [S for S in RESULT["calls"] if S["caller"] == ROOT]
        SELF.assertEqual(len(CALLS), 5)
        OVERLOADS = [S for S in CALLS if S["target_name"] == "example::overload"]
        SELF.assertEqual(len({S["callee"] for S in OVERLOADS}), 2)
        TEMPLATE = next(S for S in CALLS if S["target_name"] == "example::identity")
        SELF.assertEqual(TEMPLATE["callee"], SELF.named(RESULT, "example::identity")["id"])
        SELF.assertIn(TEMPLATE["resolution"], ("identity", "declaration_location"))
        EXTERNAL = next(S for S in CALLS if S["target_name"] == "external_function")
        SELF.assertIsNone(EXTERNAL["callee"])
        SELF.assertEqual(EXTERNAL["resolution"], "external")
        SELF.assertIn(SELF.named(RESULT, "example::leaf")["id"], RESULT["groups"][0]["functions"])
        SELF.assertIn(EXTERNAL["id"], RESULT["groups"][0]["open_calls"])
        SELF.assertIn(SELF.named(RESULT, "example::dynamic")["id"], RESULT["unassigned_functions"])
        SELF.assertEqual(len(RESULT["review_queue"]), len(RESULT["inventory"]["functions"]))

    def test_indirect_virtual_dependent_and_lambda_are_visible(SELF):
        RESULT = SELF.graph(("example::indirect", "example::dynamic", "example::dependent", "example::lambda_caller"))
        CALLS = RESULT["calls"]
        SELF.assertTrue(any(S["dispatch"] == "indirect" for S in CALLS))
        REF = SELF.named(RESULT, "example::indirect_ref")["id"]
        SELF.assertTrue(any(S["caller"] == REF and S["dispatch"] == "indirect" for S in CALLS))
        SELF.assertTrue(any(S["dispatch"] == "virtual" and S["callee"] for S in CALLS))
        SELF.assertTrue(any(S["dispatch"] == "unresolved" for S in CALLS))
        LEAF = SELF.named(RESULT, "example::leaf")["id"]
        FUNCTIONS = {F["id"]: F for F in RESULT["inventory"]["functions"]}
        SELF.assertTrue(any(S["callee"] == LEAF and FUNCTIONS[S["caller"]]["name"] == "operator()" for S in CALLS))
        SELF.assertTrue(any(S["callee"] == LEAF and FUNCTIONS[S["caller"]]["name"] == "lambda_capture" for S in CALLS))
        SELF.assertIn("flowchart TD", GRAPH.markdown(RESULT))
        SELF.assertIn("-.->", GRAPH.markdown(RESULT))
        DEPENDENT = SELF.graph(("example::dependent_known",))
        SELF.assertIn(SELF.named(DEPENDENT, "example::identity")["id"], DEPENDENT["groups"][0]["functions"])
        SELF.assertTrue(any(S["dispatch"] == "overload_candidates" and S["candidates"] for S in DEPENDENT["calls"]))

    def test_profile_specific_edges_and_stable_review_ids(SELF):
        FIRST = SELF.graph(("example::variant",))
        VARIANT = SELF.named(FIRST, "example::variant")["id"]
        CALLS = [S for S in FIRST["calls"] if S["caller"] == VARIANT]
        SELF.assertEqual({S["target_name"] for S in CALLS}, {"example::selected_zero", "example::selected_one"})
        SELF.assertTrue(all(len(S["translation_units"]) == 1 for S in CALLS))
        SECOND = SELF.graph(("example::variant",), FIRST)
        SELF.assertEqual([R["review_id"] for R in FIRST["review_queue"]], [R["review_id"] for R in SECOND["review_queue"]])
        SELF.assertFalse(any(R["requires_recheck"] for R in SECOND["review_queue"]))
        SELF.assertTrue(all(R["status"] == "unreviewed" for R in SECOND["review_queue"]))

    def test_callee_change_invalidates_callers_and_dependency_scope(SELF):
        FIRST = SELF.graph()
        SELF.HEADER.write_text(SELF.HEADER.read_text().replace("return x + 1;", "return x + 2;"))
        SECOND = SELF.graph(PREVIOUS=FIRST)
        ROWS = {R["function"]: R for R in SECOND["review_queue"]}
        ROOT = SELF.named(SECOND, "example::root")["id"]
        SELF.assertIn("reachable_callee_changed", ROWS[ROOT]["change_reasons"])
        SELF.assertTrue(ROWS[ROOT]["requires_recheck"])

    def test_removed_edges_functions_and_profiles_remain_visible(SELF):
        FIRST = SELF.graph()
        SELF.HEADER.write_text(SELF.HEADER.read_text().replace("inline int leaf(int x) { return x + 1; }", "").replace("return leaf(x);", "return x;").replace("return leaf(1);", "return 1;").replace("x = leaf(2)", "x = 2"))
        SECOND = SELF.graph(PREVIOUS=FIRST)
        SELF.assertIn(SELF.named(FIRST, "example::leaf")["id"], SECOND["removed_functions"])
        SELF.assertTrue(any("reachable_callee_changed" in R["change_reasons"] for R in SECOND["review_queue"]))
        SELF.write_database(VARIANTS=(0,), SOURCES=("one.cpp",))
        THIRD = SELF.graph(PREVIOUS=SECOND)
        SELF.assertTrue(all("tool_configuration_or_profile_changed" in R["change_reasons"] for R in THIRD["review_queue"]))

    def test_external_dependency_and_tool_change_invalidates(SELF):
        FIRST = SELF.graph()
        (SELF.ROOT / "external/external.hpp").write_text("inline int external_function() { return 8; }\n")
        SECOND = SELF.graph(PREVIOUS=FIRST)
        SELF.assertTrue(all("translation_unit_dependency_changed" in R["change_reasons"] for R in SECOND["review_queue"]))
        FIRST["tool_sha256"]["call_graph.py"] = "different"
        FIRST["fingerprint"] = GRAPH.AST.digest(GRAPH.AST.canonical(GRAPH.graph_payload(FIRST)))
        THIRD = SELF.graph(PREVIOUS=FIRST)
        SELF.assertTrue(all("tool_configuration_or_profile_changed" in R["change_reasons"] for R in THIRD["review_queue"]))

    def test_unknown_entry_corrupt_snapshot_and_parse_error_fail(SELF):
        with SELF.assertRaisesRegex(ValueError, "entry not found"):
            SELF.graph(("nonexistent",))
        RESULT = SELF.graph()
        RESULT["calls"].clear()
        with SELF.assertRaisesRegex(ValueError, "fingerprint"):
            SELF.graph(PREVIOUS=RESULT)
        SELF.HEADER.write_text(SELF.HEADER.read_text() + "invalid C++;")
        with SELF.assertRaisesRegex(ValueError, "no partial inventory"):
            SELF.graph()

    def test_tool_change_during_extraction_fails(SELF):
        from unittest.mock import patch
        INVENTORY = SELF.graph()["inventory"]
        TOOL = SELF.ROOT / "graph_tool.py"
        TOOL.write_text("before")

        def changed_inventory(*ARGS):
            TOOL.write_text("after")
            return INVENTORY

        with patch.object(GRAPH, "__file__", str(TOOL)), patch.object(GRAPH.AST, "inventory", side_effect=changed_inventory):
            with SELF.assertRaisesRegex(ValueError, "tool changed during extraction"):
                GRAPH.build_graph(SELF.ROOT, SELF.CONFIG, [SELF.DATABASE], [])

    def test_cli_snapshot_freshness_and_no_input_overwrite(SELF):
        OUTPUT = SELF.ROOT / "graph.json"
        ARGS = ["call_graph.py", "--repo", str(SELF.ROOT), "--config", "config.json", "--compile-commands", str(SELF.DATABASE), "--entry", "example::root", "--output", str(OUTPUT)]
        from unittest.mock import patch
        with patch.object(sys, "argv", ARGS), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            SELF.assertEqual(GRAPH.main(), 0)
        SELF.assertTrue(OUTPUT.with_suffix(".md").is_file())
        with patch.object(sys, "argv", ["call_graph.py", "--repo", str(SELF.ROOT), "--check", str(OUTPUT)]):
            SELF.assertEqual(GRAPH.main(), 0)
        SELF.HEADER.write_text(SELF.HEADER.read_text() + "\n// changed\n")
        with patch.object(sys, "argv", ["call_graph.py", "--repo", str(SELF.ROOT), "--check", str(OUTPUT)]), contextlib.redirect_stderr(io.StringIO()):
            SELF.assertEqual(GRAPH.main(), 1)
        DATABASE = SELF.DATABASE.read_bytes()
        with patch.object(sys, "argv", [*ARGS[:-1], str(SELF.DATABASE)]), contextlib.redirect_stderr(io.StringIO()):
            SELF.assertEqual(GRAPH.main(), 1)
        SELF.assertEqual(SELF.DATABASE.read_bytes(), DATABASE)


if __name__ == "__main__":
    unittest.main(defaultTest="CallGraphTests", verbosity=2)
