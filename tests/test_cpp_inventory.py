#!/usr/bin/env python3
"""Offline tests using real Clang ASTs; no verifier, agent or network calls."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest import mock

sys.dont_write_bytecode = True

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("inventory_cpp", REPO / "scripts/inventory_cpp.py")
INVENTORY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INVENTORY)

HEADER = """#pragma once
#include <external.hpp>
namespace example {
struct Value {
    Value() = default;
    ~Value() = default;
    int get() const;
    int operator+(int x) const { return x; }
    explicit operator bool() const { return true; }
};
inline int Value::get() const { return 1; }
inline int overload(int x) { return x; }
inline double overload(double x) { return x; }
template<class T> T identity(T x) { return x; }
template<class T> struct Box { T get(T x) { return x; } };
inline int squared_norm(int x) { return x*x; }
// int comment_only();
inline int body_anchor(int x) { return squared_norm(x); }
inline int default_argument(int x = squared_norm(0)) { return x; }
inline auto closure() { return [](int x) { return x; }; }
namespace { int internal_helper() { return 0; } }
#if CONFIG_VARIANT
using Result = double;
inline int selected_one() { return 1; }
#else
using Result = int;
inline int selected_zero() { return 0; }
#endif
inline auto variant_result() { return Result{}; }
}
"""


class InventoryTests(unittest.TestCase):
    def setUp(SELF):
        SELF.TEMP = tempfile.TemporaryDirectory(prefix="formal-eskf-ast-test.")
        SELF.addCleanup(SELF.TEMP.cleanup)
        SELF.ROOT = Path(SELF.TEMP.name)
        (SELF.ROOT / "include").mkdir()
        (SELF.ROOT / "external").mkdir()
        (SELF.ROOT / "external/external.hpp").write_text("inline int external_function() { return 7; }\n")
        (SELF.ROOT / "include/api.hpp").write_text(HEADER)
        (SELF.ROOT / "include/unused.hpp").write_text("inline int not_in_any_tu() { return 0; }\n")
        (SELF.ROOT / "one.cpp").write_text('#include "api.hpp"\nint main() { return example::identity(0); }\n')
        (SELF.ROOT / "two.cpp").write_text('#include "api.hpp"\n')
        SELF.CONFIG = SELF.ROOT / "config.json"
        SELF.CONFIG.write_text(json.dumps({"version": 1, "source_roots": ["include"],
                                          "extensions": [".hpp"],
                                          "tracked_macros": ["CONFIG_VARIANT"]}))
        SELF.DRIVER = shutil.which("clang++-18") or shutil.which("clang++")
        if SELF.DRIVER is None:
            SELF.fail("Clang driver required for AST regression tests")
        SELF.DATABASE = SELF.ROOT / "compile_commands.json"
        SELF.write_database()

    def write_database(SELF, VARIANTS=(0, 1), SOURCES=("one.cpp", "two.cpp")):
        ENTRIES = [{"directory": str(SELF.ROOT), "file": SOURCE,
                    "arguments": [SELF.DRIVER, "-std=c++20", "-Iinclude", "-Iexternal",
                                  f"-DCONFIG_VARIANT={VARIANT}", "-c", SOURCE, "-o", SOURCE + ".o"]}
                   for VARIANT in VARIANTS for SOURCE in SOURCES]
        SELF.DATABASE.write_text(json.dumps(ENTRIES))
        return ENTRIES

    def run_inventory(SELF, DATABASES=None):
        with contextlib.redirect_stderr(io.StringIO()):
            return INVENTORY.inventory(SELF.ROOT, SELF.CONFIG, DATABASES or [SELF.DATABASE])

    def test_missing_clang_reports_apt_dependencies(SELF):
        with mock.patch.dict(sys.modules, {"clang": None}), \
                SELF.assertRaisesRegex(ValueError, "sudo apt install clang-18 python3-clang-18"):
            SELF.run_inventory()

    def test_real_ast_identity_and_scope(SELF):
        RESULT = SELF.run_inventory()
        FUNCTIONS = RESULT["functions"]
        NAMES = [F["name"] for F in FUNCTIONS]
        SELF.assertEqual(NAMES.count("overload"), 2)
        SELF.assertEqual(NAMES.count("identity"), 1)
        SELF.assertEqual(NAMES.count("squared_norm"), 1)
        SELF.assertNotIn("comment_only", NAMES)
        SELF.assertIn("Value", NAMES)
        SELF.assertIn("~Value", NAMES)
        SELF.assertIn("operator+", NAMES)
        SELF.assertIn("operator bool", NAMES)
        SELF.assertIn("operator()", NAMES)
        SELF.assertIn("internal_helper", NAMES)
        SELF.assertIn("selected_zero", NAMES)
        SELF.assertIn("selected_one", NAMES)
        SELF.assertNotIn("external_function", NAMES)
        SELF.assertNotIn("not_in_any_tu", NAMES)
        SELF.assertEqual(len({F["id"] for F in FUNCTIONS}), len(FUNCTIONS))
        VALUE_GET = next(F for F in FUNCTIONS if F["qualified_name"] == "example::Value::get")
        SELF.assertEqual(len(VALUE_GET["declarations"]), 2)
        SELF.assertTrue(any(D["definition"] for D in VALUE_GET["declarations"]))
        SELF.assertEqual({len(D["translation_units"]) for D in VALUE_GET["declarations"]}, {4})
        SELF.assertEqual([F["file"] for F in RESULT["files"] if not F["parsed"]], ["include/unused.hpp"])
        SELF.assertEqual(RESULT["summary"], {"source_files": 2, "parsed_files": 1, "functions": len(FUNCTIONS)})
        SELF.assertNotIn("references", RESULT)
        SELF.assertNotIn("review_queue", RESULT)
        SELF.assertTrue(all("candidate_requirements" not in F for F in FUNCTIONS))

    def test_determinism_duplicate_units_and_reordered_databases(SELF):
        FIRST = SELF.run_inventory()
        SECOND = SELF.run_inventory([SELF.DATABASE, SELF.DATABASE])
        SELF.assertEqual(FIRST, SECOND)
        ENTRIES = json.loads(SELF.DATABASE.read_text())
        SELF.DATABASE.write_text(json.dumps(list(reversed(ENTRIES))))
        THIRD = SELF.run_inventory()
        SELF.assertEqual(FIRST["functions"], THIRD["functions"])
        SELF.assertEqual(FIRST["translation_units"], THIRD["translation_units"])
        VARIANT = next(F for F in THIRD["functions"] if F["name"] == "variant_result")
        SELF.assertEqual(len(VARIANT["signatures"]), 2)

    def test_file_local_identities_do_not_collide_on_header_basename(SELF):
        (SELF.ROOT / "include/other").mkdir()
        (SELF.ROOT / "include/other/api.hpp").write_text("namespace example { namespace { int internal_helper() { return 3; } } }\n")
        (SELF.ROOT / "two.cpp").write_text('#include "other/api.hpp"\n')
        RESULT = SELF.run_inventory()
        HELPERS = [F for F in RESULT["functions"] if F["name"] == "internal_helper"]
        SELF.assertEqual(len(HELPERS), 2)
        SELF.assertNotEqual(HELPERS[0]["id"], HELPERS[1]["id"])

    def test_only_supplied_configuration_is_inventoried(SELF):
        SELF.write_database(VARIANTS=(0,))
        RESULT = SELF.run_inventory()
        SELF.assertEqual({U["macros"]["CONFIG_VARIANT"] for U in RESULT["translation_units"]}, {"0"})
        SELF.assertNotIn("selected_one", [F["name"] for F in RESULT["functions"]])

    def test_requirement_and_proof_files_are_not_inputs(SELF):
        FIRST = SELF.run_inventory()
        DIRECTORY = SELF.ROOT / "formal/agent-review"
        DIRECTORY.mkdir(parents=True)
        (DIRECTORY / "traceability-map.json").write_text("not valid JSON")
        (SELF.ROOT / "proof.lean").write_text("not a proof")
        INVENTORY.validate_snapshot(SELF.ROOT, FIRST)
        SELF.assertEqual(SELF.run_inventory(), FIRST)

    def test_compiler_errors_are_not_partial_success(SELF):
        (SELF.ROOT / "include/api.hpp").write_text(HEADER + "invalid C++ !!\n")
        with SELF.assertRaisesRegex(ValueError, "AST parse failed"):
            SELF.run_inventory()

    def test_missing_dependency_is_not_partial_success(SELF):
        (SELF.ROOT / "include/api.hpp").write_text(HEADER + '#include "missing.hpp"\n')
        with SELF.assertRaisesRegex(ValueError, "AST parse failed"):
            SELF.run_inventory()

    def test_source_and_dependency_changes_invalidate_fingerprint(SELF):
        FIRST = SELF.run_inventory()
        (SELF.ROOT / "external/external.hpp").write_text("inline int external_function() { return 8; }\n")
        SECOND = SELF.run_inventory()
        SELF.assertNotEqual(FIRST["fingerprint"], SECOND["fingerprint"])
        SELF.assertEqual(FIRST["functions"], SECOND["functions"])
        (SELF.ROOT / "include/api.hpp").write_text(HEADER.replace("return x*x", "return x+x"))
        THIRD = SELF.run_inventory()
        SELF.assertNotEqual(SECOND["fingerprint"], THIRD["fingerprint"])
        SELF.assertEqual([F["id"] for F in SECOND["functions"]], [F["id"] for F in THIRD["functions"]])
        SELF.assertNotEqual(SECOND["functions"], THIRD["functions"])

    def test_empty_database_and_empty_scope_rejected(SELF):
        SELF.DATABASE.write_text("[]")
        with SELF.assertRaisesRegex(ValueError, "empty/invalid compilation database"):
            SELF.run_inventory()
        SELF.write_database()
        CONFIG = json.loads(SELF.CONFIG.read_text())
        CONFIG["extensions"] = [".absent"]
        SELF.CONFIG.write_text(json.dumps(CONFIG))
        with SELF.assertRaisesRegex(ValueError, "empty source manifest"):
            SELF.run_inventory()

    def test_missing_source_root_and_unsafe_paths(SELF):
        CONFIG = json.loads(SELF.CONFIG.read_text())
        CONFIG["source_roots"] = ["missing"]
        SELF.CONFIG.write_text(json.dumps(CONFIG))
        with SELF.assertRaisesRegex(ValueError, "missing source root"):
            SELF.run_inventory()
        for PATH in ("../escape", "/tmp", "include/../../escape"):
            with SELF.assertRaises(ValueError):
                INVENTORY.repository_path(SELF.ROOT, PATH)

    def test_command_quoting_and_no_shell_execution(SELF):
        ENTRY = {"directory": str(SELF.ROOT), "file": "one.cpp",
                 "command": f"{SELF.DRIVER} -I'include with spaces' -c one.cpp -o one.o"}
        _, _, _, ARGS = INVENTORY.compile_arguments(ENTRY)
        SELF.assertEqual(ARGS, ["-Iinclude with spaces"])
        for COMMAND in ("sh -c anything", "ccache clang++ one.cpp", "clang++ @arguments", "clang++ -Xclang -load plugin"):
            with SELF.assertRaises(ValueError):
                INVENTORY.compile_arguments({**ENTRY, "command": COMMAND})

    def test_atomic_json_output(SELF):
        OUTPUT = SELF.ROOT / "inventory.json"
        INVENTORY.write_output(OUTPUT, {"old": True})
        with SELF.assertRaises(TypeError):
            INVENTORY.write_output(OUTPUT, {"invalid": object()})
        SELF.assertEqual(json.loads(OUTPUT.read_text()), {"old": True})
        INVENTORY.write_output(OUTPUT, {"new": True})
        SELF.assertEqual(json.loads(OUTPUT.read_text()), {"new": True})

    def test_freshness_includes_new_files_and_dependencies(SELF):
        RESULT = SELF.run_inventory()
        INVENTORY.validate_snapshot(SELF.ROOT, RESULT)
        (SELF.ROOT / "include/new.hpp").write_text("void newly_added();\n")
        with SELF.assertRaisesRegex(ValueError, "source manifest changed"):
            INVENTORY.validate_snapshot(SELF.ROOT, RESULT)
        (SELF.ROOT / "include/new.hpp").unlink()
        (SELF.ROOT / "external/external.hpp").write_text("void changed_dependency();\n")
        with SELF.assertRaisesRegex(ValueError, "stale AST input"):
            INVENTORY.validate_snapshot(SELF.ROOT, RESULT)
        RESULT["summary"]["functions"] = 999
        with SELF.assertRaisesRegex(ValueError, "invalid inventory fingerprint"):
            INVENTORY.validate_snapshot(SELF.ROOT, RESULT)

    def test_cli_exports_symbols_without_review_files(SELF):
        OUTPUT = SELF.ROOT / "inventory.json"
        ARGUMENTS = ["inventory_cpp.py", "--repo", str(SELF.ROOT), "--config", "config.json",
                     "--compile-commands", str(SELF.DATABASE), "--output", str(OUTPUT)]
        with mock.patch.object(sys, "argv", ARGUMENTS), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            SELF.assertEqual(INVENTORY.main(), 0)
        RESULT = json.loads(OUTPUT.read_text())
        SELF.assertTrue(RESULT["functions"])
        SELF.assertNotIn("review_queue", RESULT)
        with mock.patch.object(sys, "argv", ["inventory_cpp.py", "--repo", str(SELF.ROOT), "--check", str(OUTPUT)]), \
                contextlib.redirect_stdout(io.StringIO()):
            SELF.assertEqual(INVENTORY.main(), 0)


if __name__ == "__main__":
    unittest.main()
