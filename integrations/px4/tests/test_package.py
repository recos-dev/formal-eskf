#!/usr/bin/env python3
"""Exercise package installation in disposable checkouts of the pinned PX4 commit."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

INTEGRATION_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = INTEGRATION_DIR.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(INTEGRATION_DIR / "scripts"))
from install import BOARD, MODULE, PX4_COMMIT, RECEIPT


SCRIPT_DIR = INTEGRATION_DIR / "scripts"
HOST_FILES = (
    "ROMFS/px4fmu_common/init.d-posix/rcS",
    "src/modules/logger/logged_topics.cpp",
    "src/modules/simulation/simulator_sih/CMakeLists.txt",
)


class PackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(CLS):
        CLS.TEMP = tempfile.TemporaryDirectory(prefix="formal-eskf-package-")
        CLS.ROOT = Path(CLS.TEMP.name)
        CLS.PACKAGE = CLS.ROOT / "portable-package"
        subprocess.run([sys.executable, str(SCRIPT_DIR / "install.py"), "--export", str(CLS.PACKAGE)],
                       check=True, capture_output=True)

    @classmethod
    def tearDownClass(CLS):
        CLS.TEMP.cleanup()

    def setUp(SELF):
        SELF.PX4 = SELF.ROOT / SELF._testMethodName
        # Alternates also work for a shallow source; git clone may copy its full pack.
        subprocess.run(["git", "init", "--quiet", str(SELF.PX4)], check=True)
        OBJECTS = subprocess.check_output(
            ["git", "-C", str(PX4_SOURCE), "rev-parse", "--path-format=absolute", "--git-path", "objects"],
            text=True).strip()
        (SELF.PX4 / ".git/objects/info/alternates").write_text(OBJECTS + "\n")
        subprocess.run(["git", "-C", str(SELF.PX4), "update-ref", "HEAD", PX4_COMMIT], check=True)
        subprocess.run(["git", "-C", str(SELF.PX4), "checkout", PX4_COMMIT, "--", *HOST_FILES],
                       check=True, capture_output=True)

    def install(SELF, *ARGS, PACKAGE=None, SUCCESS=True):
        RESULT = subprocess.run([sys.executable, str((PACKAGE or SELF.PACKAGE) / "install.py"),
                                 "--px4", str(SELF.PX4), *ARGS], capture_output=True, text=True)
        if SUCCESS:
            SELF.assertEqual(RESULT.returncode, 0, RESULT.stdout + RESULT.stderr)
        else:
            SELF.assertNotEqual(RESULT.returncode, 0)
        return RESULT

    def snapshot(SELF):
        return {str(FILE.relative_to(SELF.PX4)): FILE.read_bytes()
                for FILE in SELF.PX4.rglob("*") if FILE.is_file() and ".git" not in FILE.parts}

    def test_standalone_install_and_repeat(SELF):
        SELF.assertFalse((SELF.PACKAGE.parent / "formal-eskf").exists())
        UNRELATED = SELF.PX4 / "local-note.txt"
        UNRELATED.write_text("Preserve unrelated work.\n")
        SELF.install()
        MANIFEST = json.loads((SELF.PX4 / RECEIPT).read_text())
        for NAME, HASH in MANIFEST["files"].items():
            SELF.assertEqual(hashlib.sha256((SELF.PX4 / NAME).read_bytes()).hexdigest(), HASH)
        SELF.assertIn("core/include", (SELF.PX4 / MODULE / "CMakeLists.txt").read_text())
        SELF.assertNotIn("FORMAL_ESKF_ROOT", (SELF.PX4 / MODULE / "CMakeLists.txt").read_text())
        BUFFER = "include/formal_eskf/runtime/ring_buffer.hpp"
        SELF.assertEqual((SELF.PX4 / MODULE / "core" / BUFFER).read_bytes(),
                         (REPO_DIR / BUFFER).read_bytes())
        BEFORE = SELF.snapshot()
        SELF.install()
        SELF.install("--check")
        SELF.assertEqual(SELF.snapshot(), BEFORE)
        SELF.assertEqual(UNRELATED.read_text(), "Preserve unrelated work.\n")

    def test_local_edits_are_preserved(SELF):
        SELF.install()
        for NAME in (f"{MODULE}/FormalEskf.cpp", BOARD):
            FILE = SELF.PX4 / NAME
            ORIGINAL = FILE.read_bytes()
            FILE.write_bytes(ORIGINAL + b"\n// Local change\n")
            BEFORE = SELF.snapshot()
            SELF.install(SUCCESS=False)
            SELF.assertEqual(SELF.snapshot(), BEFORE)
            FILE.write_bytes(ORIGINAL)

    def test_conflicting_host_patch_preserves_all_files(SELF):
        FILE = SELF.PX4 / HOST_FILES[0]
        FILE.write_text(FILE.read_text().replace("# state estimator selection", "# Custom estimator selection"))
        BEFORE = SELF.snapshot()
        SELF.install(SUCCESS=False)
        SELF.assertEqual(SELF.snapshot(), BEFORE)

    def test_unknown_module_file_is_preserved(SELF):
        FILE = SELF.PX4 / MODULE / "local.cpp"
        FILE.parent.mkdir(parents=True)
        FILE.write_text("// Local implementation\n")
        BEFORE = SELF.snapshot()
        SELF.install(SUCCESS=False)
        SELF.assertEqual(SELF.snapshot(), BEFORE)

    def test_tampered_export_is_rejected(SELF):
        PACKAGE = SELF.ROOT / "tampered-package"
        shutil.copytree(SELF.PACKAGE, PACKAGE)
        FILE = PACKAGE / "overlay" / MODULE / "FormalEskf.cpp"
        FILE.write_text(FILE.read_text() + "\n// Modified after export\n")
        BEFORE = SELF.snapshot()
        SELF.install(PACKAGE=PACKAGE, SUCCESS=False)
        SELF.assertEqual(SELF.snapshot(), BEFORE)

    def test_export_excludes_editor_files(SELF):
        REPO = SELF.ROOT / "source-with-editor-files"
        SOURCE = REPO / "integrations/px4"
        SOURCE.mkdir(parents=True)
        for NAME in ("scripts", "configs", "patches", "module", "tests"):
            shutil.copytree(INTEGRATION_DIR / NAME, SOURCE / NAME)
        shutil.copytree(REPO_DIR / "include", REPO / "include")
        shutil.copy2(REPO_DIR / "LICENSE", REPO / "LICENSE")
        (SOURCE / "module/.FormalEskf.cpp.swp").write_bytes(b"Editor scratch data\x00")
        (REPO / "include/formal_eskf/.eskf.hpp.swp").write_bytes(b"Editor scratch data\x00")
        PACKAGE = REPO / "export"
        subprocess.run([sys.executable, str(SOURCE / "scripts/install.py"), "--export", str(PACKAGE)],
                       check=True, capture_output=True)
        EXPECTED = json.loads((SELF.PACKAGE / "manifest.json").read_text())["files"]
        ACTUAL = json.loads((PACKAGE / "manifest.json").read_text())["files"]
        SELF.assertEqual(ACTUAL, EXPECTED)

    def test_owned_update_and_removed_file(SELF):
        SELF.install()
        PACKAGE = SELF.ROOT / "updated-package"
        shutil.copytree(SELF.PACKAGE, PACKAGE)
        MANIFEST = json.loads((PACKAGE / "manifest.json").read_text())
        NAME = f"{MODULE}/FormalEskf.cpp"
        FILE = PACKAGE / "overlay" / NAME
        FILE.write_text(FILE.read_text() + "\n// Updated package version\n")
        MANIFEST["files"][NAME] = hashlib.sha256(FILE.read_bytes()).hexdigest()
        REMOVED = f"{MODULE}/core/include/formal_eskf/linalg/backend/eigen.hpp"
        SELF.assertIn(REMOVED, MANIFEST["files"])
        del MANIFEST["files"][REMOVED]
        (PACKAGE / "overlay" / REMOVED).unlink()
        (PACKAGE / "manifest.json").write_text(json.dumps(MANIFEST))
        SELF.install(PACKAGE=PACKAGE)
        SELF.install("--check", PACKAGE=PACKAGE)
        SELF.assertEqual((SELF.PX4 / NAME).read_bytes(), FILE.read_bytes())
        SELF.assertFalse((SELF.PX4 / REMOVED).exists())

    def test_wrong_revision_is_rejected(SELF):
        # Create an empty local child commit without touching the real PX4 repository.
        TREE = subprocess.check_output(["git", "-C", str(SELF.PX4), "rev-parse", "HEAD^{tree}"], text=True).strip()
        NEW_HEAD = subprocess.check_output([
            "git", "-C", str(SELF.PX4), "-c", "user.name=Package Test", "-c", "user.email=test@example.invalid",
            "commit-tree", TREE, "-p", PX4_COMMIT, "-m", "Disposable package test fixture"], text=True).strip()
        subprocess.run(["git", "-C", str(SELF.PX4), "update-ref", "HEAD", NEW_HEAD], check=True)
        BEFORE = SELF.snapshot()
        RESULT = SELF.install(SUCCESS=False)
        SELF.assertIn("PX4 must be at", RESULT.stderr)
        SELF.assertEqual(SELF.snapshot(), BEFORE)


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--px4", required=True, type=Path, help="Local pinned PX4 checkout; read only")
    ARGS = PARSER.parse_args()
    PX4_SOURCE = ARGS.px4.resolve()
    HEAD = subprocess.check_output(["git", "-C", str(PX4_SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if HEAD != PX4_COMMIT:
        PARSER.error(f"PX4 source must be at {PX4_COMMIT}")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
