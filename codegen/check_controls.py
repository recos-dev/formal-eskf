#!/usr/bin/env python3
"""Require real Lean/ESBMC rejection of deliberately corrupted codegen artifacts."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

from generate import PINNED_HEADER, REPO_DIR
from ins.evaluation import digest, write_json
from ins.verify import require
from ins.verify.lean import export


def check(COMMAND, DIRECTORY, NAME, EXPECTED, CWD=None, ENV=None):
    LOG_PATH = DIRECTORY / (NAME + '.log')
    with LOG_PATH.open('w') as LOG_FILE:
        RESULT = subprocess.run(COMMAND, cwd=CWD or DIRECTORY, stdout=LOG_FILE,
                                stderr=subprocess.STDOUT, env=ENV, timeout=180)
    LOG = LOG_PATH.read_text()
    require(RESULT.returncode != 0 and all(TEXT in LOG for TEXT in EXPECTED),
            f'{NAME} did not produce the expected counterexample; inspect {DIRECTORY}')
    return {'command': COMMAND, 'exit_code': RESULT.returncode, 'expected_diagnostics': EXPECTED,
            'log_sha256': digest(DIRECTORY / (NAME + '.log'))}


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('--output', type=Path, default=REPO_DIR / 'output/codegen/controls')
    PARSER.add_argument('--esbmc', default=os.environ.get('FORMAL_ESKF_ESBMC') or
                        shutil.which('esbmc') or str(REPO_DIR / 'tools/esbmc-8.4.0/bin/esbmc'))
    PARSER.add_argument('--lake', default=shutil.which('lake') or str(REPO_DIR / 'tools/elan/bin/lake'))
    ARGS = PARSER.parse_args()
    OUTPUT = ARGS.output.resolve()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'result.json').unlink(missing_ok=True)
    HEADER = (REPO_DIR / PINNED_HEADER).read_text()
    require(HEADER.count('F(0U, 3U)') == 1, 'unexpected source mutation point')
    BAD_INDEX = OUTPUT / PINNED_HEADER
    BAD_INDEX.parent.mkdir(parents=True, exist_ok=True)
    BAD_INDEX.write_text(HEADER.replace('F(0U, 3U)', 'F(15U, 3U)', 1))
    ESBMC = check([ARGS.esbmc, str(REPO_DIR / 'formal/cpp/esbmc/codegen.cpp'),
                  '--std', 'c++20', '--function', 'verify_codegen_memory', '--unwind', '16',
                  '--quiet', '--timeout', '90s', '--memlimit', '3g',
                  '-I', str(OUTPUT / 'include'), '-I', str(REPO_DIR / 'include')],
                 OUTPUT, 'esbmc-negative', ['VERIFICATION FAILED', 'generated read is in bounds'])
    # Export the changed parsed DAG, then ask Lean to prove it against its unchanged model.
    export(HEADER.replace('F(0U, 3U)', '(-F(0U, 3U))', 1), OUTPUT / 'lean')
    # One false identity suffices; do not spend resources rejecting all 15 rows.
    ROW = (OUTPUT / 'lean/Row00.lean').read_text()
    (OUTPUT / 'lean/Negative.lean').write_text(ROW[:ROW.index('def generated_0_1 ')] + 'end\nend Codegen\n')
    COMMAND = [ARGS.lake, 'env', 'lean', '-j1', str(OUTPUT / 'lean/Negative.lean')]
    ENV = {**os.environ, 'LEAN_NUM_THREADS': '1'}
    if Path(ARGS.lake) == REPO_DIR / 'tools/elan/bin/lake':
        ENV['ELAN_HOME'] = str(REPO_DIR / 'tools/elan')
    LEAN = check(COMMAND, OUTPUT, 'lean-negative', ['unsolved goals'], REPO_DIR / 'formal/lean', ENV)
    write_json(OUTPUT / 'result.json', {'header_sha256': digest(REPO_DIR / PINNED_HEADER),
                                      'esbmc_rejected_bad_index': ESBMC, 'lean_rejected_bad_sign': LEAN})
    print('Negative controls: Lean rejected the sign error; ESBMC rejected the out-of-bounds read')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError) as ERROR:
        sys.exit(f'error: {ERROR}')
