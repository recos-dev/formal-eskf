#!/usr/bin/env python3
"""Generate, format and independently verify the standalone INS products."""

import argparse
import hashlib
import json
import platform
from pathlib import Path
import sympy

from ins.verify import require
from ins.verify.cpp import operation_counts
from ins.products.schedule import kernels
from ins.products.cpp import header
from ins.verify.product_reference import contracts
from ins.verify.product_source import parse_header, verify
from ins.formatting import fingerprint


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('--check', action='store_true')
    ARGS = PARSER.parse_args()
    ROOT = Path(__file__).resolve().parents[3]
    DEST = ROOT / 'include/formal_eskf/eskf/generated'
    SPECS = contracts()
    KERNELS = kernels()
    require({K['name'] for K in KERNELS} == SPECS.keys(), 'kernel inventory mismatch')
    REPORT = {}
    TEXTS = {}
    for GROUP in ('reset', 'innovation', 'joseph'):
        TEXT = header([K for K in KERNELS if K['group'] == GROUP])
        PROGRAMS = parse_header(TEXT, GROUP)
        for NAME, PROGRAM in PROGRAMS.items():
            COUNT = verify(PROGRAM, SPECS[NAME])
            REPORT[NAME] = {'identities': COUNT, 'assignments': len(PROGRAM),
                            'scalar_operations_before_compilation': operation_counts(PROGRAM)}
            print(NAME, COUNT, 'identities', flush=True)
        FILE = DEST / f'ins_{GROUP}.hpp'
        if ARGS.check:
            if FILE.read_text() != TEXT:
                raise ValueError('stale generated header: ' + str(FILE))
        TEXTS[FILE] = TEXT
    # Validate every product before publishing any pinned header.
    if not ARGS.check:
        for FILE, TEXT in TEXTS.items():
            FILE.write_text(TEXT)
    OUTPUT = ROOT / 'output/codegen/products'
    OUTPUT.mkdir(parents=True, exist_ok=True)
    SOURCES = list((ROOT / 'codegen/ins').rglob('*.py')) + [Path(__file__).resolve()]
    SOURCES += list(DEST.glob('ins_*.hpp'))
    SOURCES += [ROOT / 'codegen/products.py', ROOT / 'codegen/requirements.txt']
    SOURCES += [P for P in (ROOT / 'include/formal_eskf').rglob('*.hpp') if P.parent != DEST]
    SOURCES += list((ROOT / 'docs/spec/requirements/core').glob('*.md'))
    REPORT = {
        'kernels': REPORT,
        'python': platform.python_version(),
        'sympy': sympy.__version__,
        'formatter': fingerprint(),
        'sources': {
            str(F.relative_to(ROOT)): hashlib.sha256(F.read_bytes()).hexdigest()
            for F in sorted(SOURCES)
        },
    }
    (OUTPUT / 'algebra.json').write_text(json.dumps(REPORT, indent=2) + '\n')


if __name__ == '__main__':
    main()
