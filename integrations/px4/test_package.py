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

from install import BOARD, MODULE, PX4_COMMIT, RECEIPT


SCRIPT_DIR = Path(__file__).resolve().parent
HOST_FILES = (
    "ROMFS/px4fmu_common/init.d-posix/rcS",
    "src/modules/logger/logged_topics.cpp",
    "src/modules/simulation/simulator_sih/CMakeLists.txt",
)


class PackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.TEMP = tempfile.TemporaryDirectory(prefix="formal-eskf-package-")
        cls.ROOT = Path(cls.TEMP.name)
        cls.PACKAGE = cls.ROOT / "portable-package"
        subprocess.run([sys.executable, str(SCRIPT_DIR / "install.py"), "--export", str(cls.PACKAGE)],
                       check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.TEMP.cleanup()

    def setUp(self):
        self.PX4 = self.ROOT / self._testMethodName
        # Alternates also work for a shallow source; git clone may copy its full pack.
        subprocess.run(["git", "init", "--quiet", str(self.PX4)], check=True)
        OBJECTS = subprocess.check_output(
            ["git", "-C", str(PX4_SOURCE), "rev-parse", "--path-format=absolute", "--git-path", "objects"],
            text=True).strip()
        (self.PX4 / ".git/objects/info/alternates").write_text(OBJECTS + "\n")
        subprocess.run(["git", "-C", str(self.PX4), "update-ref", "HEAD", PX4_COMMIT], check=True)
        subprocess.run(["git", "-C", str(self.PX4), "checkout", PX4_COMMIT, "--", *HOST_FILES],
                       check=True, capture_output=True)

    def install(self, *ARGS, PACKAGE=None, SUCCESS=True):
        RESULT = subprocess.run([sys.executable, str((PACKAGE or self.PACKAGE) / "install.py"),
                                 "--px4", str(self.PX4), *ARGS], capture_output=True, text=True)
        if SUCCESS:
            self.assertEqual(RESULT.returncode, 0, RESULT.stdout + RESULT.stderr)
        else:
            self.assertNotEqual(RESULT.returncode, 0)
        return RESULT

    def snapshot(self):
        return {str(FILE.relative_to(self.PX4)): FILE.read_bytes()
                for FILE in self.PX4.rglob("*") if FILE.is_file() and ".git" not in FILE.parts}

    def test_standalone_install_and_repeat(self):
        self.assertFalse((self.PACKAGE.parent / "formal-eskf").exists())
        UNRELATED = self.PX4 / "local-note.txt"
        UNRELATED.write_text("Preserve unrelated work.\n")
        self.install()
        MANIFEST = json.loads((self.PX4 / RECEIPT).read_text())
        for NAME, HASH in MANIFEST["files"].items():
            self.assertEqual(hashlib.sha256((self.PX4 / NAME).read_bytes()).hexdigest(), HASH)
        self.assertIn("core/include", (self.PX4 / MODULE / "CMakeLists.txt").read_text())
        self.assertNotIn("FORMAL_ESKF_ROOT", (self.PX4 / MODULE / "CMakeLists.txt").read_text())
        BEFORE = self.snapshot()
        self.install()
        self.install("--check")
        self.assertEqual(self.snapshot(), BEFORE)
        self.assertEqual(UNRELATED.read_text(), "Preserve unrelated work.\n")

    def test_local_edits_are_preserved(self):
        self.install()
        for NAME in (f"{MODULE}/FormalEskf.cpp", BOARD):
            FILE = self.PX4 / NAME
            ORIGINAL = FILE.read_bytes()
            FILE.write_bytes(ORIGINAL + b"\n// Local change\n")
            BEFORE = self.snapshot()
            self.install(SUCCESS=False)
            self.assertEqual(self.snapshot(), BEFORE)
            FILE.write_bytes(ORIGINAL)

    def test_conflicting_host_patch_preserves_all_files(self):
        FILE = self.PX4 / HOST_FILES[0]
        FILE.write_text(FILE.read_text().replace("# state estimator selection", "# Custom estimator selection"))
        BEFORE = self.snapshot()
        self.install(SUCCESS=False)
        self.assertEqual(self.snapshot(), BEFORE)

    def test_unknown_module_file_is_preserved(self):
        FILE = self.PX4 / MODULE / "local.cpp"
        FILE.parent.mkdir(parents=True)
        FILE.write_text("// Local implementation\n")
        BEFORE = self.snapshot()
        self.install(SUCCESS=False)
        self.assertEqual(self.snapshot(), BEFORE)

    def test_tampered_export_is_rejected(self):
        PACKAGE = self.ROOT / "tampered-package"
        shutil.copytree(self.PACKAGE, PACKAGE)
        FILE = PACKAGE / "overlay" / MODULE / "FormalEskf.cpp"
        FILE.write_text(FILE.read_text() + "\n// Modified after export\n")
        BEFORE = self.snapshot()
        self.install(PACKAGE=PACKAGE, SUCCESS=False)
        self.assertEqual(self.snapshot(), BEFORE)

    def test_export_excludes_editor_files(self):
        REPO = self.ROOT / "source-with-editor-files"
        SOURCE = REPO / "integrations/px4"
        SOURCE.mkdir(parents=True)
        for NAME in ("install.py", "px4-v1.16.2.patch", "formal_eskf.px4board"):
            shutil.copy2(SCRIPT_DIR / NAME, SOURCE / NAME)
        shutil.copytree(SCRIPT_DIR / "module", SOURCE / "module")
        shutil.copytree(SCRIPT_DIR.parents[1] / "include", REPO / "include")
        shutil.copy2(SCRIPT_DIR.parents[1] / "LICENSE", REPO / "LICENSE")
        (SOURCE / "module/.FormalEskf.cpp.swp").write_bytes(b"Editor scratch data\x00")
        (REPO / "include/formal_eskf/.eskf.hpp.swp").write_bytes(b"Editor scratch data\x00")
        PACKAGE = REPO / "export"
        subprocess.run([sys.executable, str(SOURCE / "install.py"), "--export", str(PACKAGE)],
                       check=True, capture_output=True)
        EXPECTED = json.loads((self.PACKAGE / "manifest.json").read_text())["files"]
        ACTUAL = json.loads((PACKAGE / "manifest.json").read_text())["files"]
        self.assertEqual(ACTUAL, EXPECTED)

    def test_owned_update_and_removed_file(self):
        self.install()
        PACKAGE = self.ROOT / "updated-package"
        shutil.copytree(self.PACKAGE, PACKAGE)
        MANIFEST = json.loads((PACKAGE / "manifest.json").read_text())
        NAME = f"{MODULE}/FormalEskf.cpp"
        FILE = PACKAGE / "overlay" / NAME
        FILE.write_text(FILE.read_text() + "\n// Updated package version\n")
        MANIFEST["files"][NAME] = hashlib.sha256(FILE.read_bytes()).hexdigest()
        REMOVED = f"{MODULE}/core/include/formal_eskf/linalg/backend/eigen.hpp"
        self.assertIn(REMOVED, MANIFEST["files"])
        del MANIFEST["files"][REMOVED]
        (PACKAGE / "overlay" / REMOVED).unlink()
        (PACKAGE / "manifest.json").write_text(json.dumps(MANIFEST))
        self.install(PACKAGE=PACKAGE)
        self.install("--check", PACKAGE=PACKAGE)
        self.assertEqual((self.PX4 / NAME).read_bytes(), FILE.read_bytes())
        self.assertFalse((self.PX4 / REMOVED).exists())

    def test_wrong_revision_is_rejected(self):
        # Create an empty local child commit without touching the real PX4 repository.
        TREE = subprocess.check_output(["git", "-C", str(self.PX4), "rev-parse", "HEAD^{tree}"], text=True).strip()
        NEW_HEAD = subprocess.check_output([
            "git", "-C", str(self.PX4), "-c", "user.name=Package Test", "-c", "user.email=test@example.invalid",
            "commit-tree", TREE, "-p", PX4_COMMIT, "-m", "Disposable package test fixture"], text=True).strip()
        subprocess.run(["git", "-C", str(self.PX4), "update-ref", "HEAD", NEW_HEAD], check=True)
        BEFORE = self.snapshot()
        RESULT = self.install(SUCCESS=False)
        self.assertIn("PX4 must be at", RESULT.stderr)
        self.assertEqual(self.snapshot(), BEFORE)


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--px4", required=True, type=Path, help="Local pinned PX4 checkout; read only")
    ARGS = PARSER.parse_args()
    PX4_SOURCE = ARGS.px4.resolve()
    HEAD = subprocess.check_output(["git", "-C", str(PX4_SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if HEAD != PX4_COMMIT:
        PARSER.error(f"PX4 source must be at {PX4_COMMIT}")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
