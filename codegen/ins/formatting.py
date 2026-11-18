"""Format emitted C++ with the repository style before verification/publication."""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess

from .verify import require

REPO_DIR = Path(__file__).resolve().parents[2]
STYLE = REPO_DIR / '.clang-format'


def formatter():
    NAME = os.environ.get('CLANG_FORMAT', 'clang-format-14')
    EXECUTABLE = shutil.which(NAME)
    require(EXECUTABLE is not None, f'{NAME} not found; install clang-format-14 or set CLANG_FORMAT')
    return EXECUTABLE


def format_cpp(TEXT):
    RESULT = subprocess.run(
        [formatter(), '--style=file', '--assume-filename=' + str(REPO_DIR / 'ins_covariance.hpp')],
        input=TEXT, capture_output=True, text=True, check=True,
    )
    return RESULT.stdout


def fingerprint():
    EXECUTABLE = formatter()
    return {
        'version': subprocess.check_output([EXECUTABLE, '--version'], text=True).strip(),
        'binary_sha256': hashlib.sha256(Path(EXECUTABLE).read_bytes()).hexdigest(),
        'style_sha256': hashlib.sha256(STYLE.read_bytes()).hexdigest(),
    }
