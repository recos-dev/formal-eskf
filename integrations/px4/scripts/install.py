#!/usr/bin/env python3
"""Copy a self-contained formal-eskf module into pinned PX4, or export a package."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tempfile


PX4_COMMIT = "54f0455ffcd755534539a7cf33a09a20bf71d29d"
MODULE = "src/modules/formal_eskf"
BOARD = "boards/px4/sitl/formal_eskf.px4board"
RECEIPT = f"{MODULE}/package-manifest.json"
SCRIPT_DIR = Path(__file__).resolve().parent
MODULE_FILES = (
    "CMakeLists.txt", "FormalEskf.cpp", "FormalEskf.hpp", "FormalEskfNavigation.cpp",
    "FormalEskfAttitude.cpp", "FormalEskfOutput.cpp", "DeploymentProfile.hpp",
    "DeploymentConfig.hpp",
    "MagneticModel.hpp", "HeightAiding.hpp", "InnovationRatio.hpp",
    "ImuPrediction.hpp", "ImuHistory.hpp", "OutputPredictor.hpp", "ObservationHistory.hpp", "Kconfig", "formal_eskf_params.c",
)
HEADER_SUFFIXES = (".h", ".hpp", ".hxx", ".inl", ".tpp")


def digest(DATA):
    return hashlib.sha256(DATA).hexdigest()


def git(PX4, *ARGS, INPUT=None):
    return subprocess.run(["git", "-C", str(PX4), *ARGS], input=INPUT, capture_output=True)


def owned_path(NAME):
    PARTS = PurePosixPath(NAME)
    if (PARTS.is_absolute() or ".." in PARTS.parts or str(PARTS) != NAME
            or not (NAME.startswith(MODULE + "/") or NAME == BOARD)
            or NAME == RECEIPT):
        raise ValueError(f"Invalid package path: {NAME}")
    return NAME


def load_exported_package():
    """Validate an export before using its payload or host patch."""
    PATCH = (SCRIPT_DIR / "px4-v1.16.2.patch").read_bytes()
    MANIFEST = json.loads((SCRIPT_DIR / "manifest.json").read_text())
    if MANIFEST["schema_version"] != 1 or MANIFEST["px4_commit"] != PX4_COMMIT:
        raise ValueError("Unsupported package version or PX4 revision")
    FILES = {owned_path(NAME): (SCRIPT_DIR / "overlay" / NAME).read_bytes()
             for NAME in MANIFEST["files"]}
    if any(digest(DATA) != MANIFEST["files"][NAME] for NAME, DATA in FILES.items()):
        raise ValueError("Exported package content does not match its manifest")
    if digest(PATCH) != MANIFEST["patch_sha256"]:
        raise ValueError("Exported PX4 patch does not match its manifest")
    return MANIFEST, FILES, PATCH


def configure_module(REPO_DIR, CONFIG_DATA):
    """Generate a deployment header from a stable snapshot of the selected config."""
    with tempfile.TemporaryDirectory(prefix="formal-eskf-config-") as TEMP:
        SNAPSHOT = Path(TEMP) / ".config"
        SNAPSHOT.write_bytes(CONFIG_DATA)
        RESULT = subprocess.run(
            [sys.executable, str(REPO_DIR / "scripts/configure.py"), "px4-header", "--config", str(SNAPSHOT)],
            capture_output=True,
        )
    if RESULT.returncode:
        raise ValueError(RESULT.stderr.decode().strip())
    return RESULT.stdout


def collect_sources(CONFIG):
    """Map repository sources to their installed PX4 paths."""
    INTEGRATION_DIR = SCRIPT_DIR.parent
    REPO_DIR = INTEGRATION_DIR.parents[1]
    PATCH = (INTEGRATION_DIR / "patches/px4-v1.16.2.patch").read_bytes()
    FILES = {f"{MODULE}/{NAME}": (INTEGRATION_DIR / "module" / NAME).read_bytes() for NAME in MODULE_FILES}
    FILES[f"{MODULE}/tests/ModuleTest.cpp"] = (INTEGRATION_DIR / "tests/ModuleTest.cpp").read_bytes()
    if CONFIG is not None:
        CONFIG_DATA = CONFIG.read_bytes()
        FILES[f"{MODULE}/DeploymentConfig.hpp"] = configure_module(REPO_DIR, CONFIG_DATA)
        FILES[f"{MODULE}/configuration.config"] = CONFIG_DATA
    for FILE in sorted((REPO_DIR / "include/formal_eskf").rglob("*")):
        if FILE.is_file() and FILE.suffix in HEADER_SUFFIXES and not FILE.name.startswith("."):
            FILES[f"{MODULE}/core/include/{FILE.relative_to(REPO_DIR / 'include').as_posix()}"] = FILE.read_bytes()
    FILES[f"{MODULE}/core/LICENSE"] = (REPO_DIR / "LICENSE").read_bytes()
    FILES[BOARD] = (INTEGRATION_DIR / "configs/formal_eskf.px4board").read_bytes()
    REVISION = git(REPO_DIR, "rev-parse", "HEAD")
    STATUS = git(REPO_DIR, "status", "--porcelain")
    MANIFEST = {
        "schema_version": 1,
        "px4_commit": PX4_COMMIT,
        "formal_eskf_commit": REVISION.stdout.decode().strip() if REVISION.returncode == 0 else None,
        "formal_eskf_dirty": bool(STATUS.stdout) if STATUS.returncode == 0 else None,
        "patch_sha256": digest(PATCH),
        "files": {NAME: digest(DATA) for NAME, DATA in sorted(FILES.items())},
    }
    return MANIFEST, FILES, PATCH


def source_package(CONFIG=None):
    if (SCRIPT_DIR / "manifest.json").exists():
        if CONFIG is not None:
            raise ValueError("Configure the package in the source repository before export")
        return load_exported_package()
    return collect_sources(CONFIG)


def json_bytes(VALUE):
    return (json.dumps(VALUE, indent=2, sort_keys=True) + "\n").encode()


def export_package(OUTPUT, MANIFEST, FILES, PATCH):
    # A fresh directory prevents overwriting a source checkout or previous export.
    OUTPUT.mkdir(parents=True, exist_ok=False)
    for NAME, DATA in FILES.items():
        DEST = OUTPUT / "overlay" / NAME
        DEST.parent.mkdir(parents=True, exist_ok=True)
        DEST.write_bytes(DATA)
    (OUTPUT / "manifest.json").write_bytes(json_bytes(MANIFEST))
    (OUTPUT / "px4-v1.16.2.patch").write_bytes(PATCH)
    shutil.copy2(Path(__file__), OUTPUT / "install.py")
    (OUTPUT / "README.txt").write_text(
        "Self-contained formal-eskf PX4 package\n\n"
        "Requires PX4 v1.16.2 at " + PX4_COMMIT + ".\n"
        "Install: python3 install.py --px4 /path/to/PX4-Autopilot\n"
        "Build: make -C /path/to/PX4-Autopilot px4_sitl_formal_eskf -j2\n"
        "The original formal-eskf checkout is not required.\n"
    )
    print(f"Exported {len(FILES)} files to {OUTPUT}")


def destination(PX4, NAME):
    DEST = PX4 / NAME
    for PART in (DEST, *DEST.parents):
        if PART == PX4:
            break
        if PART.is_symlink():
            raise ValueError(f"Refusing a symlink in destination: {PART}")
    if DEST.exists() and not DEST.is_file():
        raise ValueError(f"Expected a regular file: {DEST}")
    return DEST


def validate_checkout(PX4):
    TOP = git(PX4, "rev-parse", "--show-toplevel")
    HEAD = git(PX4, "rev-parse", "HEAD")
    if TOP.returncode or Path(TOP.stdout.decode().strip()).resolve() != PX4:
        raise ValueError("--px4 must name the root of a PX4 Git checkout")
    if HEAD.stdout.decode().strip() != PX4_COMMIT:
        raise ValueError(f"PX4 must be at {PX4_COMMIT}; existing work was preserved")


def check_owned_files(PX4, MANIFEST, FILES):
    """Preflight every managed destination before any mutation."""
    MARKER = destination(PX4, RECEIPT)
    PREVIOUS = json.loads(MARKER.read_text()) if MARKER.exists() else {"files": {}}
    OLD_FILES = {owned_path(NAME): HASH for NAME, HASH in PREVIOUS["files"].items()}
    # Only previously managed files may be replaced or removed. Preserve edits.
    for NAME in set(FILES) | set(OLD_FILES):
        DEST = destination(PX4, NAME)
        if DEST.exists():
            CURRENT = digest(DEST.read_bytes())
            if CURRENT not in (OLD_FILES.get(NAME), MANIFEST["files"].get(NAME)):
                raise ValueError(f"Local changes or unowned file at {DEST}; move or reconcile it first")
    for FILE in (PX4 / MODULE).rglob("*"):
        NAME = FILE.relative_to(PX4).as_posix()
        if FILE.is_symlink() or (FILE.is_file() and NAME not in FILES and NAME not in OLD_FILES and NAME != RECEIPT):
            raise ValueError(f"Unmanaged file in module: {FILE}")

    return MARKER, PREVIOUS, OLD_FILES


def check_patch(PX4, PATCH):
    APPLIED = git(PX4, "apply", "--reverse", "--check", "-", INPUT=PATCH).returncode == 0
    if not APPLIED:
        RESULT = git(PX4, "apply", "--check", "-", INPUT=PATCH)
        if RESULT.returncode:
            raise ValueError("PX4 host patch conflicts; existing work was preserved:\n" + RESULT.stderr.decode())
    return APPLIED


def install_package(PX4, MANIFEST, FILES, PATCH, CHECK):
    validate_checkout(PX4)
    MARKER, PREVIOUS, OLD_FILES = check_owned_files(PX4, MANIFEST, FILES)
    APPLIED = check_patch(PX4, PATCH)
    if CHECK:
        MATCHES = MARKER.exists() and PREVIOUS == MANIFEST and APPLIED
        MATCHES = MATCHES and all((PX4 / NAME).is_file() and digest((PX4 / NAME).read_bytes()) == HASH
                                  for NAME, HASH in MANIFEST["files"].items())
        if not MATCHES:
            raise ValueError("Installed package differs from this source/export; run installation first")
        print(f"Verified {len(FILES)} installed files and PX4 host patch")
        return

    if not APPLIED:
        RESULT = git(PX4, "apply", "-", INPUT=PATCH)
        if RESULT.returncode:
            raise ValueError(RESULT.stderr.decode())
    for NAME in OLD_FILES.keys() - FILES.keys():
        (PX4 / NAME).unlink(missing_ok=True)
    for NAME, DATA in FILES.items():
        DEST = PX4 / NAME
        if not DEST.exists() or DEST.read_bytes() != DATA:
            DEST.parent.mkdir(parents=True, exist_ok=True)
            DEST.write_bytes(DATA)
    MARKER.write_bytes(json_bytes(MANIFEST))
    print(f"Installed {len(FILES)} files into {PX4}")
    print(f"Build: make -C {PX4} px4_sitl_formal_eskf -j2")


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    TARGET = PARSER.add_mutually_exclusive_group(required=True)
    TARGET.add_argument("--px4", type=Path, help="Pinned PX4 Git checkout to install into")
    TARGET.add_argument("--export", type=Path, help="New directory for a relocatable package")
    PARSER.add_argument("--check", action="store_true", help="Verify an existing --px4 installation without writing")
    PARSER.add_argument("--config", type=Path, help="Apply a menuconfig .config when packaging from the source repository")
    ARGS = PARSER.parse_args()
    if ARGS.check and not ARGS.px4:
        PARSER.error("--check requires --px4")
    try:
        MANIFEST, FILES, PATCH = source_package(ARGS.config.resolve() if ARGS.config else None)
        if ARGS.export:
            export_package(ARGS.export.resolve(), MANIFEST, FILES, PATCH)
        else:
            install_package(ARGS.px4.resolve(), MANIFEST, FILES, PATCH, ARGS.check)
    except (OSError, ValueError, KeyError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
