#!/usr/bin/env python3
"""Certify native products against exact rational dense equations and IEEE RNE."""

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path

from ins.verify import require
from ins.verify.numerical import decode_coefficient, evaluate_expression
from ins.verify.product_reference import contracts, LAYOUTS
from ins.verify.product_source import parse_header


def transpose(A):
    return [list(ROW) for ROW in zip(*A)]


def multiply(A, B):
    return [
        [
            sum((A[I][K] * B[K][J] for K in range(len(B)) if A[I][K] and B[K][J]), Fraction(0))
            for J in range(len(B[0]))
        ]
        for I in range(len(A))
    ]


def plus(A, B):
    return [[A[I][J] + B[I][J] for J in range(len(A[0]))] for I in range(len(A))]


def identity(N):
    return [[Fraction(I == J) for J in range(N)] for I in range(N)]


def reference(INPUTS, SPEC):
    if SPEC['kind'] == 'reset':
        G = identity(15)
        for I in range(3):
            for J in range(3):
                G[I + 6][J + 6] = INPUTS['J'][I][J]
        return multiply(multiply(G, INPUTS['P']), transpose(G))
    COLS = LAYOUTS[SPEC['layout']]
    H = [[Fraction(0) for _ in range(15)] for _ in COLS]
    if SPEC['layout'] == 'body_vector' and SPEC['kind'] != 'joseph':
        X, Y, Z = [ROW[0] for ROW in INPUTS['h']]
        HAT = [[0, -Z, Y], [Z, 0, -X], [-Y, X, 0]]
        for I in range(3):
            for J in range(3):
                H[I][J + 6] = HAT[I][J]
    else:
        for I, COL in enumerate(COLS):
            H[I][COL] = 1
    if SPEC['kind'] == 'cross':
        return multiply(INPUTS['P'], transpose(H))
    if SPEC['kind'] == 'innovation':
        return plus(multiply(H, INPUTS['C']), INPUTS['V'])
    A = INPUTS['A']
    return plus(
        multiply(multiply(A, INPUTS['P']), transpose(A)),
        multiply(multiply(INPUTS['K'], INPUTS['V']), transpose(INPUTS['K'])),
    )


def finite_words(WORDS, BITS):
    F, E = (23, 8) if BITS == 32 else (52, 11)
    require(all(type(W) is int and 0 <= W < 1 << BITS for W in WORDS), 'invalid IEEE encoding')
    return all(((W >> F) & ((1 << E) - 1)) != ((1 << E) - 1) for W in WORDS)


def check(RECORD, PROGRAM, SPEC):
    BITS = RECORD['bits']
    require(type(BITS) is int and BITS in (32, 64), 'invalid scalar type')
    VALUES, INPUTS = {}, {}
    for NAME, (R, C) in SPEC['dimensions'].items():
        require(len(RECORD[NAME]) == R * C, 'wrong input size')
        if not finite_words(RECORD[NAME], BITS):
            return {'domain': 'nonfinite_input'}
        A = [
            [decode_coefficient(RECORD[NAME][I * C + J], BITS) for J in range(C)] for I in range(R)
        ]
        if NAME in SPEC['symmetric']:
            require(A == transpose(A), 'asymmetric input outside equivalence domain')
        if NAME == 'A':
            COLS = LAYOUTS[SPEC['layout']]
            require(
                all(
                    A[I][J] == Fraction(I == J)
                    for I in range(15)
                    for J in range(15)
                    if J not in COLS
                ),
                'A violates the observation-layout contract',
            )
        INPUTS[NAME] = A
        VALUES.update(
            {(NAME, I, J): (A[I][J], Fraction(0), A[I][J]) for I in range(R) for J in range(C)}
        )
    R, C = SPEC['shape']
    require(len(RECORD['output']) == R * C and len(RECORD['dense']) == R * C, 'wrong output size')
    GENERATED_FINITE = finite_words(RECORD['output'], BITS)
    DENSE_FINITE = finite_words(RECORD['dense'], BITS)
    require(
        GENERATED_FINITE == RECORD['finite'] and DENSE_FINITE == RECORD['dense_finite'],
        'wrong finite diagnostic',
    )
    try:
        for TARGET, NODE in PROGRAM:
            VALUES[TARGET] = evaluate_expression(NODE, VALUES, BITS)
    except OverflowError:
        require(not GENERATED_FINITE, 'finite C++ disagrees with overflowing parsed DAG')
        return {'domain': 'overflow', 'dense_finite': DENSE_FINITE}
    require(GENERATED_FINITE, 'nonfinite C++ disagrees with finite parsed DAG')
    REF = reference(INPUTS, SPEC)
    MAX_ERROR = MAX_BOUND = MAX_DENSE = Fraction(0)
    MAX_RATIO = 0.0
    for I in range(R):
        for J in range(C):
            EXACT, BOUND, ROUNDED = VALUES['out', I, J]
            ACTUAL = decode_coefficient(RECORD['output'][I * C + J], BITS)
            require(EXACT == REF[I][J], 'independent rational equation mismatch')
            require(ROUNDED == ACTUAL, 'actual C++ differs from IEEE RNE simulation')
            ERROR = abs(ACTUAL - EXACT)
            require(ERROR <= BOUND, 'error bound exceeded')
            if SPEC['mirror']:
                require(
                    RECORD['output'][I * C + J] == RECORD['output'][J * C + I], 'mirror bits differ'
                )
            MAX_ERROR = max(MAX_ERROR, ERROR)
            MAX_BOUND = max(MAX_BOUND, BOUND)
            MAX_RATIO = max(MAX_RATIO, float(ERROR / BOUND) if BOUND else 0)
            if DENSE_FINITE:
                MAX_DENSE = max(
                    MAX_DENSE, abs(decode_coefficient(RECORD['dense'][I * C + J], BITS) - EXACT)
                )
    SCALE = max(abs(X) for ROW in REF for X in ROW)
    return {
        'domain': 'certified_finite',
        'max_error_to_bound': MAX_RATIO,
        'max_matrix_scaled_error': float(MAX_ERROR / SCALE) if SCALE else None,
        'max_absolute_error': float(MAX_ERROR),
        'dense_max_matrix_scaled_error': (
            float(MAX_DENSE / SCALE) if SCALE and DENSE_FINITE else None
        ),
        'max_absolute_bound': float(MAX_BOUND),
        'dense_finite': DENSE_FINITE,
    }


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('records', type=Path)
    ARGS = PARSER.parse_args()
    ROOT = Path(__file__).resolve().parents[3]
    SPECS = contracts()
    PROGRAMS = {}
    HASHES = {}
    for GROUP in ('reset', 'innovation', 'joseph'):
        FILE = ROOT / f'include/formal_eskf/eskf/generated/ins_{GROUP}.hpp'
        PROGRAMS.update(parse_header(FILE.read_text(), GROUP))
        HASHES[str(FILE.relative_to(ROOT))] = hashlib.sha256(FILE.read_bytes()).hexdigest()
    RESULTS = []
    SEEN = set()
    for LINE in ARGS.records.read_text().splitlines():
        RECORD = json.loads(LINE)
        KEY = (RECORD['kernel'], RECORD['bits'], RECORD['seed'])
        require(KEY not in SEEN, 'duplicate record')
        SEEN.add(KEY)
        RESULT = check(RECORD, PROGRAMS[RECORD['kernel']], SPECS[RECORD['kernel']])
        RESULTS.append({'kernel': KEY[0], 'bits': KEY[1], 'seed': KEY[2], **RESULT})
        if len(RESULTS) % 80 == 0:
            print('Checked', len(RESULTS), flush=True)
    require(
        SEEN == {(N, B, I) for N in SPECS for B in (32, 64) for I in range(72)},
        'incomplete native corpus',
    )
    REPORT = {
        'headers': HASHES,
        'verifiers': {
            str(P.relative_to(ROOT)): hashlib.sha256(P.read_bytes()).hexdigest()
            for P in sorted((ROOT / 'codegen/ins/verify').glob('*.py'))
        },
        'records_sha256': hashlib.sha256(ARGS.records.read_bytes()).hexdigest(),
        'scope': 'Finite recorded operations, not a uniform deployment-domain error budget.',
        'results': RESULTS,
    }
    DEST = ARGS.records.with_suffix('.certified.json')
    DEST.write_text(json.dumps(REPORT, indent=2, allow_nan=False) + '\n')
    print('Certified', len(RESULTS), 'classified records:', DEST)


if __name__ == '__main__':
    main()
