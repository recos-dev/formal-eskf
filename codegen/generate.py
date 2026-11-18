#!/usr/bin/env python3
"""Generate or check the standalone INS covariance kernel and its provenance."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import sympy

from ins.covariance import render
from ins.formatting import fingerprint, format_cpp
from ins.verify import require
from ins.verify.algebra import verify_identities
from ins.verify.cpp import parse
from ins.verify.lean import export

ROOT = Path(__file__).resolve().parent
REPO_DIR = ROOT.parent
MODEL_ID = 'ins15-right-local-truncated-full-covariance-v1'
PINNED_HEADER = Path('include/formal_eskf/eskf/generated/ins_covariance.hpp')
SPEC_FILES = (
    'docs/spec/requirements/core/covariance-prediction.md',
    'docs/spec/requirements/core/process-noise.md',
    'docs/spec/requirements/core/state.md',
    'docs/spec/requirements/common/numerical-profile.md',
    'docs/spec/requirements/common/numerical-acceptance.md',
)


def digest(PATH):
    return hashlib.sha256(PATH.read_bytes()).hexdigest()


def source_fingerprints(CORE):
    require((CORE / 'include/formal_eskf').is_dir(), 'not a formal-eskf source directory')
    # The generated artifact is fingerprinted separately, not as its own input.
    FILES = sorted(FILE for FILE in (CORE / 'include/formal_eskf').rglob('*.hpp')
                   if FILE != CORE / PINNED_HEADER)
    FILES += [CORE / NAME for NAME in SPEC_FILES]
    require(all(FILE.is_file() for FILE in FILES), 'missing source/specification dependency')
    return {str(FILE.relative_to(CORE)): digest(FILE) for FILE in FILES}


def tool_fingerprints():
    # Include the independent verifiers so a changed checker invalidates evidence.
    FILES = [ROOT / 'generate.py', ROOT / 'requirements.txt']
    FILES += sorted((ROOT / 'ins').rglob('*.py'))
    return {str(FILE.relative_to(ROOT)): digest(FILE) for FILE in FILES}


def build_manifest(CORE, TEXT):
    OPERATIONS = Counter()

    def count(NODE):
        if NODE[0] not in ('ref', 'int'):
            OPERATIONS[NODE[0]] += 1
            for CHILD in NODE[1:]:
                count(CHILD)

    for _, NODE in parse(TEXT):
        count(NODE)
    return {
        'schema_version': 3,
        'model': MODEL_ID,
        'sympy': sympy.__version__,
        'python': '.'.join(map(str, sys.version_info[:3])),
        'tool_sources': tool_fingerprints(),
        'formatter': fingerprint(),
        'core_and_specs': source_fingerprints(CORE),
        'header_sha256': hashlib.sha256(TEXT.encode('utf-8')).hexdigest(),
        'scalar_operations_before_compilation': dict(OPERATIONS),
        'shape': {
            'nominal': 16,
            'error': 15,
            'noise': 12,
            'upper_outputs': 120,
            'mirrored_outputs': 105,
        },
        'numerical_profile': {
            'rounding': 'nearest ties-to-even',
            'FMA': False,
            'fast_math': False,
            'FTZ_DAZ': False,
        },
        'preconditions': ['finite producer inputs', 'structured F', 'symmetric P', 'diagonal Q_i'],
        'conditional_psd': ['P positive semidefinite', 'nonnegative diagonal Q_i', 'exact real arithmetic'],
    }


def read_verified_header(CORE, OUTPUT):
    """Reject stale provenance, changed emission and unsupported C++ source."""
    TEXT = (OUTPUT / 'ins_covariance.hpp').read_bytes().decode('utf-8')
    STORED = json.loads((OUTPUT / 'manifest.json').read_text(encoding='utf-8'))
    require(STORED.get('header_sha256') == hashlib.sha256(TEXT.encode('utf-8')).hexdigest(),
            'stale/inconsistent header hash')
    require(
        STORED == build_manifest(CORE, TEXT),
        'stale/inconsistent manifest: regenerate and requalify',
    )
    require(TEXT == format_cpp(render()), 'generated header differs from formatted generator output')
    parse(TEXT)
    return TEXT


def parse_args():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument(
        '--pin', action='store_true',
        help='also publish/check include/formal_eskf/eskf/generated/ins_covariance.hpp',
    )
    PARSER.add_argument(
        '--core', type=Path, default=REPO_DIR,
        help='formal-eskf checkout (default: this repository)',
    )
    PARSER.add_argument(
        '--output', type=Path, default=REPO_DIR / 'output/codegen/ins',
        help='kernel and manifest directory (default: output/codegen/ins in this repository)',
    )
    PARSER.add_argument(
        '--check', action='store_true',
        help='check existing kernel and manifest without rewriting them',
    )
    PARSER.add_argument(
        '--lean', type=Path,
        help='export Lean certificates to this directory; does not run Lean',
    )
    return PARSER.parse_args()


def main():
    ARGS = parse_args()
    TEXT = read_verified_header(ARGS.core, ARGS.output) if ARGS.check else format_cpp(render())
    COUNT = verify_identities(TEXT)
    if ARGS.pin and ARGS.check:
        require((ARGS.core / PINNED_HEADER).read_bytes() == TEXT.encode('utf-8'), 'stale pinned header')
    if not ARGS.check:
        # Validate the full expression and source dependencies before publication.
        MANIFEST = build_manifest(ARGS.core, TEXT)
        ARGS.output.mkdir(parents=True, exist_ok=True)
        (ARGS.output / 'ins_covariance.hpp').write_bytes(TEXT.encode('utf-8'))
        (ARGS.output / 'manifest.json').write_text(
            json.dumps(MANIFEST, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )
        if ARGS.pin:
            DESTINATION = ARGS.core / PINNED_HEADER
            DESTINATION.parent.mkdir(parents=True, exist_ok=True)
            DESTINATION.write_bytes(TEXT.encode('utf-8'))
    if ARGS.lean:
        export(TEXT, ARGS.lean)
    print(f'INS covariance: {COUNT}/225 exact identities; complete source and manifest checked')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, SyntaxError, subprocess.CalledProcessError) as ERROR:
        sys.exit(f'error: {ERROR}')
