#!/usr/bin/env python3
"""Inventory source declarations, not proved functions or template executions.

Read trusted build-generated compilation databases without executing their shell
commands. Probe the recorded compiler for its include search path, then parse
each translation unit with libclang. Never accept a partial/error AST as complete.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile


def digest(DATA):
    return hashlib.sha256(DATA).hexdigest()


def canonical(DATA):
    return json.dumps(DATA, sort_keys=True, separators=(",", ":")).encode()


def repository_path(ROOT, VALUE):
    PATH = Path(VALUE)
    if PATH.is_absolute() or ".." in PATH.parts:
        raise ValueError(f"unsafe repository path: {VALUE}")
    PATH = (ROOT / PATH).resolve()
    if not PATH.is_relative_to(ROOT):
        raise ValueError(f"path escapes repository: {VALUE}")
    return PATH


def display_path(ROOT, PATH):
    PATH = Path(PATH).resolve()
    return PATH.relative_to(ROOT).as_posix() if PATH.is_relative_to(ROOT) else str(PATH)


def source_manifest(ROOT, CONFIG):
    SOURCES = {}
    for VALUE in CONFIG["source_roots"]:
        SOURCE_ROOT = repository_path(ROOT, VALUE)
        if not SOURCE_ROOT.is_dir():
            raise ValueError(f"missing source root: {VALUE}")
        for FILE in sorted(SOURCE_ROOT.rglob("*")):
            if FILE.is_file() and FILE.suffix in CONFIG["extensions"]:
                if not FILE.resolve().is_relative_to(ROOT):
                    raise ValueError(f"source escapes repository: {FILE}")
                SOURCES[FILE.resolve()] = FILE.read_bytes()
    if not SOURCES:
        raise ValueError("empty source manifest")
    return SOURCES


def validate_snapshot(ROOT, RESULT):
    if RESULT.get("fingerprint") != digest(canonical({K: V for K, V in RESULT.items() if K != "fingerprint"})):
        raise ValueError("invalid inventory fingerprint")
    CURRENT = {display_path(ROOT, F) for F in source_manifest(ROOT, RESULT["scope"])}
    if CURRENT != {F["file"] for F in RESULT["files"]}:
        raise ValueError("source manifest changed since AST extraction")
    for INPUT in RESULT["inputs"]:
        FILE = ROOT / INPUT["file"]
        if digest(FILE.read_bytes()) != INPUT["sha256"]:
            raise ValueError(f"stale AST input: {INPUT['file']}")


def compile_arguments(ENTRY):
    ARGS = ENTRY.get("arguments")
    if ARGS is None:
        ARGS = shlex.split(ENTRY["command"])
    if not isinstance(ARGS, list) or not ARGS or not all(isinstance(A, str) for A in ARGS):
        raise ValueError("invalid compilation arguments")
    DRIVER = ARGS[0]
    if not re.fullmatch(r"(?:[\w.+-]+-)?(?:c\+\+|g\+\+|clang\+\+)(?:-[\d.]+)?", Path(DRIVER).name):
        raise ValueError(f"unsupported compiler/launcher: {DRIVER}; use a direct C++ compiler command")
    DIRECTORY = Path(ENTRY["directory"]).resolve()
    SOURCE = (DIRECTORY / ENTRY["file"]).resolve()
    RESULT = []
    INDEX = 1
    while INDEX < len(ARGS):
        ARG = ARGS[INDEX]
        if ARG in ("-o", "-MF", "-MT", "-MQ", "-MJ"):
            if INDEX + 1 == len(ARGS):
                raise ValueError(f"missing argument after {ARG}")
            INDEX += 2
            continue
        if ARG in ("-c", "-MD", "-MMD", "-MP"):
            INDEX += 1
            continue
        if ARG.startswith(("@", "-fplugin", "-Xclang", "-include-pch", "-fmodule", "-save-temps")):
            raise ValueError(f"unsupported inventory compiler option: {ARG}")
        if not ARG.startswith("-") and (DIRECTORY / ARG).resolve() == SOURCE:
            INDEX += 1
            continue
        if ARG.startswith(("-o", "-MF", "-MT", "-MQ", "-MJ")):
            raise ValueError(f"use a separate compiler option/value: {ARG}")
        RESULT.append(ARG)
        INDEX += 1
    return DRIVER, DIRECTORY, SOURCE, RESULT


def compiler_environment(DRIVER, DIRECTORY, ARGS):
    # libclang wheels do not necessarily discover the host C++ standard library.
    # Preserve the recorded target/sysroot/defines; do not invent include paths.
    RESULT = subprocess.run(
        [DRIVER, *ARGS, "-E", "-x", "c++", "-", "-v", "-dM"],
        cwd=DIRECTORY, input="", text=True, capture_output=True, timeout=60,
        env={**os.environ, "LC_ALL": "C"}, check=False,
    )
    if RESULT.returncode:
        raise ValueError(f"compiler include-path probe failed:\n{RESULT.stderr}")
    MATCH = re.search(r"#include <\.\.\.> search starts here:\n(.*?)End of search list\.", RESULT.stderr, re.S)
    if not MATCH:
        raise ValueError("compiler did not report its include search path")
    INCLUDES = [LINE.strip() for LINE in MATCH[1].splitlines() if LINE.strip()]
    if any("(framework directory)" in PATH for PATH in INCLUDES):
        raise ValueError("framework include paths are not supported by this inventory profile")
    MACROS = dict(re.findall(r"^#define (\w+) (.*)$", RESULT.stdout, re.M))
    VERSION = subprocess.run([DRIVER, "--version"], cwd=DIRECTORY, text=True,
                             capture_output=True, timeout=10, check=True).stdout.splitlines()[0]
    return INCLUDES, MACROS, VERSION


def clang_resource_directory(VERSION):
    # Use builtin headers matching the AST frontend, not GCC's intrinsic headers
    # or a different installed Clang major. The original driver supplies libstdc++.
    MAJOR = re.search(r"clang version (\d+)", VERSION)
    if not MAJOR:
        raise ValueError(f"cannot identify libclang version: {VERSION}")
    DRIVER = shutil.which(f"clang++-{MAJOR[1]}") or shutil.which("clang++")
    if not DRIVER:
        raise ValueError(f"install clang-{MAJOR[1]} for the matching builtin headers")
    TEXT = subprocess.check_output([DRIVER, "--version"], text=True, timeout=10)
    if not re.search(rf"clang version {MAJOR[1]}\.", TEXT):
        raise ValueError(f"install clang-{MAJOR[1]}; {DRIVER} does not match libclang")
    RESOURCE = subprocess.check_output([DRIVER, "-print-resource-dir"], text=True, timeout=10).strip()
    if not (Path(RESOURCE) / "include").is_dir():
        raise ValueError(f"missing Clang builtin headers: {RESOURCE}")
    return RESOURCE


def qualified_name(CURSOR):
    PARTS = [CURSOR.spelling]
    PARENT = CURSOR.semantic_parent
    while PARENT and PARENT.kind.name != "TRANSLATION_UNIT":
        PARTS.append(PARENT.displayname or "(anonymous)")
        PARENT = PARENT.semantic_parent
    return "::".join(reversed(PARTS))


def extract_functions(TU, ROOT, DIRECTORY, SOURCES, PROFILE, FUNCTIONS):
    FUNCTION_KINDS = {"FUNCTION_DECL", "FUNCTION_TEMPLATE", "CXX_METHOD",
                      "CONSTRUCTOR", "DESTRUCTOR", "CONVERSION_FUNCTION"}

    def visit(CURSOR):
        FILE = (DIRECTORY / CURSOR.location.file.name).resolve() if CURSOR.location.file else None
        if FILE is not None and FILE not in SOURCES:
            return
        KIND = CURSOR.kind.name
        if KIND == "LAMBDA_EXPR":
            # libclang's ordinary walk omits the closure's call operator.
            for METHOD in CURSOR.type.get_declaration().get_children():
                if METHOD.spelling == "operator()":
                    visit(METHOD)
            return
        if FILE in SOURCES and KIND in FUNCTION_KINDS:
            USR = CURSOR.get_usr().replace(str(ROOT) + "/", "")
            if not USR:
                raise ValueError(f"function without Clang identity: {CURSOR.location}")
            # File-local USRs can contain only the basename. Two distinct
            # headers named api.hpp must not merge their internal helpers.
            FILE_LOCAL = CURSOR.linkage.name in ("INTERNAL", "NO_LINKAGE", "UNIQUE_EXTERNAL") or USR.startswith(f"c:{FILE.name}@")
            ID = f"{display_path(ROOT, FILE)}::{USR}" if FILE_LOCAL else USR
            FUNCTION = FUNCTIONS.setdefault(ID, {
                "id": ID, "usr": USR, "name": CURSOR.spelling, "qualified_name": qualified_name(CURSOR),
                "kind": KIND, "declarations": [],
            })
            LOCATION = {"file": display_path(ROOT, FILE), "line": CURSOR.location.line,
                        "column": CURSOR.location.column,
                        "start": CURSOR.extent.start.offset, "end": CURSOR.extent.end.offset,
                        "definition": CURSOR.is_definition(),
                        "signature": CURSOR.type.spelling or CURSOR.displayname,
                        "canonical_signature": CURSOR.type.get_canonical().spelling or CURSOR.displayname,
                        "source_sha256": digest(SOURCES[FILE][CURSOR.extent.start.offset:CURSOR.extent.end.offset])}
            EXISTING = next((D for D in FUNCTION["declarations"]
                             if all(D[K] == V for K, V in LOCATION.items())), None)
            if EXISTING is None:
                EXISTING = {**LOCATION, "translation_units": []}
                FUNCTION["declarations"].append(EXISTING)
            if PROFILE not in EXISTING["translation_units"]:
                EXISTING["translation_units"].append(PROFILE)
        for CHILD in CURSOR.get_children():
            visit(CHILD)

    visit(TU.cursor)


def inventory(ROOT, CONFIG_PATH, DATABASES):
    try:
        from clang import cindex
    except ImportError as ERROR:
        raise ValueError("Clang Python bindings are required: sudo apt install clang-18 python3-clang-18") from ERROR
    CONFIG = json.loads(CONFIG_PATH.read_text())
    if CONFIG.get("version") != 1 or not CONFIG.get("source_roots"):
        raise ValueError("invalid AST inventory configuration")
    SOURCES = source_manifest(ROOT, CONFIG)
    try:
        INDEX = cindex.Index.create()
    except cindex.LibclangError as ERROR:
        raise ValueError(f"could not load libclang: {ERROR}") from ERROR
    VERSION_FN = cindex.conf.lib.clang_getClangVersion
    VERSION_FN.restype = cindex._CXString
    VERSION_FN.errcheck = cindex._CXString.from_result
    VERSION = VERSION_FN()
    RESOURCE = clang_resource_directory(VERSION)
    FUNCTIONS, ENVIRONMENTS, UNITS = {}, {}, {}
    PARSED = set()
    SNAPSHOT = {str(CONFIG_PATH.resolve()): digest(CONFIG_PATH.read_bytes()),
                str(Path(__file__).resolve()): digest(Path(__file__).read_bytes())}
    for FILE, DATA in SOURCES.items():
        SNAPSHOT[str(FILE)] = digest(DATA)
    for DATABASE in sorted(set(Path(D).resolve() for D in DATABASES)):
        DATA = DATABASE.read_bytes()
        SNAPSHOT[str(DATABASE)] = digest(DATA)
        ENTRIES = json.loads(DATA)
        if not isinstance(ENTRIES, list) or not ENTRIES:
            raise ValueError(f"empty/invalid compilation database: {DATABASE}")
        for ENTRY in ENTRIES:
            DRIVER, DIRECTORY, SOURCE, ARGS = compile_arguments(ENTRY)
            PROFILE = digest(canonical([display_path(ROOT, SOURCE), str(DIRECTORY), DRIVER, ARGS]))
            if PROFILE in UNITS:
                continue
            KEY = (DRIVER, str(DIRECTORY), *ARGS)
            if KEY not in ENVIRONMENTS:
                ENVIRONMENTS[KEY] = compiler_environment(DRIVER, DIRECTORY, ARGS)
            INCLUDES, MACROS, DRIVER_VERSION = ENVIRONMENTS[KEY]
            EFFECTIVE = [*ARGS, f"-working-directory={DIRECTORY}", f"-resource-dir={RESOURCE}",
                         "-isystem", str(Path(RESOURCE) / "include"),
                         *[ARG for PATH in INCLUDES for ARG in ("-isystem", PATH)]]
            print(f"AST: {display_path(ROOT, SOURCE)}", file=sys.stderr)
            try:
                TU = INDEX.parse(str(SOURCE), args=EFFECTIVE)
            except cindex.TranslationUnitLoadError as ERROR:
                raise ValueError(f"could not load translation unit: {SOURCE}") from ERROR
            ERRORS = [str(D) for D in TU.diagnostics if D.severity >= cindex.Diagnostic.Error]
            if ERRORS:
                raise ValueError("AST parse failed (no partial inventory accepted):\n" + "\n".join(ERRORS))
            FILES = {(DIRECTORY / I.include.name).resolve() for I in TU.get_includes()} | {SOURCE}
            for FILE in sorted(FILES):
                HASH = digest(FILE.read_bytes())
                if str(FILE) in SNAPSHOT and SNAPSHOT[str(FILE)] != HASH:
                    raise ValueError(f"source changed during inventory: {FILE}")
                SNAPSHOT[str(FILE)] = HASH
                if FILE in SOURCES:
                    PARSED.add(display_path(ROOT, FILE))
            UNITS[PROFILE] = {"id": PROFILE, "file": display_path(ROOT, SOURCE),
                              "directory": str(DIRECTORY), "compiler": DRIVER_VERSION,
                              "arguments": [DRIVER, *ARGS], "parse_arguments": EFFECTIVE,
                              "macros": {K: MACROS.get(K) for K in CONFIG.get("tracked_macros", [])},
                              "dependencies": sorted(display_path(ROOT, F) for F in FILES),
                              "diagnostics": [str(D) for D in TU.diagnostics]}
            extract_functions(TU, ROOT, DIRECTORY, SOURCES, PROFILE, FUNCTIONS)
    if not FUNCTIONS:
        raise ValueError("no project functions discovered; check database/source scope")
    for FUNCTION in FUNCTIONS.values():
        FUNCTION["declarations"].sort(key=lambda D: (D["file"], D["line"], D["column"], D["canonical_signature"], D["signature"], D["source_sha256"]))
        FUNCTION["signatures"] = sorted({D["canonical_signature"] for D in FUNCTION["declarations"]})
        for DECL in FUNCTION["declarations"]:
            DECL["translation_units"].sort()
    for FILE, HASH in SNAPSHOT.items():
        if digest(Path(FILE).read_bytes()) != HASH:
            raise ValueError(f"input changed during inventory: {FILE}")
    RESULT = {
        "version": 1, "tool": {"name": "libclang", "version": VERSION},
        "scope": {"source_roots": CONFIG["source_roots"], "extensions": CONFIG["extensions"], "kind": "source_declarations",
                  "limitations": ["Not proof coverage or an execution/call graph.",
                                  "Template declarations are not exhaustive scalar, dimension, backend or instantiation coverage.",
                                  "Only supplied compilation databases are parsed; unparsed project files remain in the file manifest.",
                                  "External headers are dependency hashes, not project functions or verified internals."]},
        "inputs": [{"file": display_path(ROOT, F), "sha256": H} for F, H in sorted(SNAPSHOT.items())],
        "files": [{"file": display_path(ROOT, F), "sha256": digest(D), "parsed": display_path(ROOT, F) in PARSED}
                  for F, D in sorted(SOURCES.items())],
        "translation_units": sorted(UNITS.values(), key=lambda U: U["id"]),
        "functions": sorted(FUNCTIONS.values(), key=lambda F: (F["qualified_name"], F["id"])),
        "summary": {"source_files": len(SOURCES), "parsed_files": len(PARSED), "functions": len(FUNCTIONS)},
    }
    RESULT["fingerprint"] = digest(canonical(RESULT))
    return RESULT


def write_output(OUTPUT, RESULT):
    OUTPUT = Path(OUTPUT)
    if not OUTPUT.parent.is_dir():
        raise ValueError(f"output directory does not exist: {OUTPUT.parent}")
    # Publish only a complete successful inventory; never truncate a previous run.
    with tempfile.NamedTemporaryFile(mode="w", dir=OUTPUT.parent, encoding="utf-8", delete=False) as FILE:
        TEMP = Path(FILE.name)
        try:
            json.dump(RESULT, FILE, indent=2, ensure_ascii=False)
            FILE.write("\n")
            FILE.close()
            TEMP.replace(OUTPUT)
        finally:
            TEMP.unlink(missing_ok=True)


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    PARSER.add_argument("--config", default="configs/ast-inventory.json")
    PARSER.add_argument("--compile-commands", action="append", type=Path,
                        help="repeat for additional build configurations; no compiler shell command is evaluated")
    PARSER.add_argument("--output", type=Path)
    PARSER.add_argument("--check", type=Path, help="check inventory fingerprint and input freshness without parsing again")
    ARGS = PARSER.parse_args()
    try:
        ROOT = ARGS.repo.resolve()
        if ARGS.check:
            if ARGS.compile_commands or ARGS.output:
                raise ValueError("--check cannot be combined with generation options")
            validate_snapshot(ROOT, json.loads(ARGS.check.read_text()))
            print("AST inventory inputs: unchanged")
            return 0
        if not ARGS.compile_commands or not ARGS.output:
            raise ValueError("generation requires --compile-commands and --output")
        RESULT = inventory(ROOT, repository_path(ROOT, ARGS.config), ARGS.compile_commands)
        if any(ARGS.output.resolve() == (ROOT / I["file"]).resolve() for I in RESULT["inputs"]):
            raise ValueError("output must not overwrite an inventory input")
        write_output(ARGS.output, RESULT)
        SUMMARY = RESULT["summary"]
        print(f"AST inventory: {SUMMARY['functions']} functions; {SUMMARY['parsed_files']}/{SUMMARY['source_files']} files parsed.\n"
              f"Output: {ARGS.output}")
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
