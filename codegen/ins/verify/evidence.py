"""Check complete native inventories, rational error witnesses and side effects."""

import json
from pathlib import Path

from . import require
from .cpp import parse
from .numerical import check_record, decode_coefficient
from ..evaluation import digest, verify_builds, write_json


def read_records(PATH):
    ROWS = [json.loads(LINE) for LINE in PATH.read_text().splitlines()]
    RECORDS = {}
    for ROW in ROWS:
        KEY = (ROW['kind'], ROW['bits'], ROW.get('seed'))
        require(KEY not in RECORDS, f'duplicate native record: {KEY}')
        RECORDS[KEY] = ROW
    for BITS in (32, 64):
        EXPECTED = {('status', BITS, SEED) for SEED in range(728)}
        require({KEY for KEY in RECORDS if KEY[:2] == ('status', BITS)} == EXPECTED,
                'incomplete native status inventory')
        SUMMARY = RECORDS['contracts', BITS, None]
        require(SUMMARY['calls'] == 5824 and SUMMARY['failures'] == 0, 'incomplete/failed native contracts')
        for SEED in range(728):
            STATUS = RECORDS['status', BITS, SEED]
            if SEED < 64:
                require(STATUS['covariance'] == 0 and STATUS['predict'] == 0,
                        'ordinary valid fixture unexpectedly failed')
            if SEED in (68, 69, 70, 71, 72, 73, 74, 87):
                require(STATUS['covariance'] != 0 and STATUS['predict'] != 0,
                        'invalid fixture unexpectedly succeeded')
            # The asymmetric-prior diagnostic is explicitly outside kernel equivalence.
            EXPECTED_NUMERICAL = RECORDS['status', BITS, SEED]['covariance'] == 0 and SEED != 77
            require((('numerical', BITS, SEED) in RECORDS) == EXPECTED_NUMERICAL,
                    f'missing or unexpected numerical record: {BITS}/{SEED}')
    require(all(KEY[0] in ('status', 'contracts', 'numerical') and KEY[1] in (32, 64)
                and (KEY[2] is None if KEY[0] == 'contracts' else KEY[2] in range(728))
                for KEY in RECORDS), 'unexpected native record')
    return RECORDS


def psd_counterexample(RECORD):
    """A negative principal minor disproves PSD; its absence proves nothing."""
    P = [decode_coefficient(WORD, RECORD['bits']) for WORD in RECORD['output']]
    for I in range(15):
        if P[16 * I] < 0:
            return {'negative_diagonal': I}
        for J in range(I + 1, 15):
            DETERMINANT = P[16 * I] * P[16 * J] - P[15 * I + J] * P[15 * J + I]
            if DETERMINANT < 0:
                return {'indices': [I, J], 'determinant': str(DETERMINANT)}
    return None


def certify(OUTPUT):
    (OUTPUT / 'numerical.json').unlink(missing_ok=True)
    REPORT = verify_builds(OUTPUT)
    SNAPSHOTS = Path(REPORT['snapshot'])
    PROGRAM = parse((SNAPSHOTS / 'generated/include/formal_eskf/eskf/generated/ins_covariance.hpp').read_text())
    BUILDS = {(BUILD['backend'], BUILD['opt'], BUILD['mode'], BUILD['variant']): BUILD
              for BUILD in REPORT['builds']}
    require(len(BUILDS) == len(REPORT['builds']), 'duplicate build profile')
    CACHE = {}
    SUMMARIES = []
    CONTRACT_CALLS = 0
    for BACKEND, OPT, MODE in sorted({KEY[:3] for KEY in BUILDS}):
        DENSE, GENERATED = [read_records(Path(BUILDS[BACKEND, OPT, MODE, VARIANT]['directory']) / 'native.log')
                            for VARIANT in ('dense', 'generated')]
        CONTRACT_CALLS += sum(ROWS['contracts', BITS, None]['calls']
                              for ROWS in (DENSE, GENERATED) for BITS in (32, 64))
        for BITS in (32, 64):
            STATUS_DIFFERENCES = []
            MAXIMA = {}
            COUNT = 0
            CHANGED = 0
            WITNESSES = []
            for SEED in range(728):
                D, G = [ROWS['status', BITS, SEED] for ROWS in (DENSE, GENERATED)]
                if D != G:
                    STATUS_DIFFERENCES.append({'seed': SEED, 'dense': D, 'generated': G})
                if SEED in (68, 69, 70, 71, 72, 73, 74, 87):
                    require(D == G, 'changed pre-kernel validation status')
                G = GENERATED.get(('numerical', BITS, SEED))
                if G is None:
                    continue
                D = DENSE.get(('numerical', BITS, SEED))
                KEY = json.dumps([{K: V for K, V in ROW.items() if K not in ('seed', 'kind')} if ROW else None
                                  for ROW in (G, D)], sort_keys=True)
                if KEY not in CACHE:
                    CACHE[KEY] = check_record(G, PROGRAM, D)
                COUNT += 1
                CHANGED += D is not None and G['output'] != D['output']
                for NAME, VALUE in CACHE[KEY].items():
                    MAXIMA[NAME] = max(MAXIMA.get(NAME, 0), VALUE)
                if SEED >= 600:
                    GW = psd_counterexample(G)
                    DW = psd_counterexample(D) if D else None
                    if GW or DW:
                        WITNESSES.append({'seed': SEED, 'generated': GW, 'dense': DW})
            SUMMARIES.append({'backend': BACKEND, 'opt': OPT, 'mode': MODE, 'bits': BITS,
                              'certificates': COUNT, 'different_output_bits': CHANGED,
                              'maxima': MAXIMA, 'status_differences': STATUS_DIFFERENCES,
                              'wide_exponent_psd_counterexamples': WITNESSES})
            print(f'Certify {BACKEND} {OPT} q{MODE} binary{BITS}: {COUNT} records', flush=True)
    RESULT = {'builds_sha256': digest(OUTPUT / 'builds.json'),
              'native_calls': CONTRACT_CALLS, 'unique_certificates': len(CACHE), 'results': SUMMARIES,
              'scope': 'finite recorded executions; no universal IEEE PSD or status equivalence claim'}
    write_json(OUTPUT / 'numerical.json', RESULT)
    return RESULT
