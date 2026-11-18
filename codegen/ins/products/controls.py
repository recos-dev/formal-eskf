"""Require real checker rejection of corrupted product artifacts."""

import argparse
import os
from pathlib import Path
import shutil

from ins.evaluation import digest, write_json
from ins.verify import require
from check_controls import check

ROOT = Path(__file__).resolve().parents[3]


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('--output', type=Path, default=ROOT / 'output/codegen/products/controls')
    PARSER.add_argument('--esbmc', default=os.environ.get('FORMAL_ESKF_ESBMC') or
                        shutil.which('esbmc') or str(ROOT / 'tools/esbmc-8.4.0/bin/esbmc'))
    PARSER.add_argument('--lake', default=shutil.which('lake') or str(ROOT / 'tools/elan/bin/lake'))
    ARGS = PARSER.parse_args()
    OUTPUT = ARGS.output.resolve()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'results.json').unlink(missing_ok=True)
    HEADER = ROOT / 'include/formal_eskf/eskf/generated/ins_reset.hpp'
    BAD = OUTPUT / 'formal_eskf/eskf/generated/ins_reset.hpp'
    BAD.parent.mkdir(parents=True, exist_ok=True)
    TEXT = HEADER.read_text()
    require('P(0U, 0U)' in TEXT, 'missing mutation point')
    BAD.write_text(TEXT.replace('P(0U, 0U)', 'P(15U, 0U)', 1))
    ESBMC = check([
        ARGS.esbmc, str(ROOT / 'formal/cpp/esbmc/codegen_products.cpp'),
        '--std', 'c++20', '--function', 'verify_product_memory', '--unwind', '16',
        '--timeout', '90s', '--memlimit', '3g', '-DPROVE_INS_RESET',
        '-DFORMAL_ESKF_PROOF_BINARY64=0', '-I', str(OUTPUT), '-I', str(ROOT / 'include'),
    ], OUTPUT, 'esbmc', ['VERIFICATION FAILED', 'generated read is in bounds'])

    SOURCE = ROOT / 'formal/lean/FormalESKF/codegen/products/InsHorizontalPositionCrossRow00.lean'
    TEXT = SOURCE.read_text()
    POINT = '(P ⟨0, by decide⟩ ⟨0, by decide⟩)'
    require(POINT in TEXT, 'missing Lean mutation point')
    LEAN_BAD = OUTPUT / 'Bad.lean'
    LEAN_BAD.write_text(TEXT.replace(POINT, POINT + ' + 1', 1))
    ENV = {**os.environ, 'LEAN_NUM_THREADS': '1'}
    if Path(ARGS.lake) == ROOT / 'tools/elan/bin/lake':
        ENV['ELAN_HOME'] = str(ROOT / 'tools/elan')
    LEAN = check([ARGS.lake, 'env', 'lean', '-j1', str(LEAN_BAD)], OUTPUT, 'lean',
                 ['error: `ring_nf` made no progress on the goal'], ROOT / 'formal/lean', ENV)
    write_json(OUTPUT / 'results.json', {
        'header_sha256': digest(HEADER), 'lean_source_sha256': digest(SOURCE),
        'esbmc_rejected_bad_index': ESBMC, 'lean_rejected_bad_coefficient': LEAN,
    })
    print('Real Lean/ESBMC negative controls rejected corrupted artifacts')


if __name__ == '__main__':
    main()
