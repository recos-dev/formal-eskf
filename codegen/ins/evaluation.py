"""Build isolated INS experiments; never modify the production core."""

import hashlib
import itertools
import json
import os
from pathlib import Path
import shlex
import shutil
import statistics
import subprocess

from .verify import require

ROOT = Path(__file__).resolve().parents[1]
REPO_DIR = ROOT.parent
NATIVE_DIR = ROOT / 'tests/native'


def digest(PATH):
    return hashlib.sha256(PATH.read_bytes()).hexdigest()


def write_json(PATH, VALUE):
    PATH.write_text(json.dumps(VALUE, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')


def evaluation_sources():
    FILES = sorted((REPO_DIR / 'include').rglob('*.hpp'))
    FILES += sorted(FILE for FILE in ROOT.rglob('*') if FILE.suffix in ('.py', '.hpp', '.cpp', '.patch', '.txt'))
    return {str(FILE.relative_to(REPO_DIR)): digest(FILE) for FILE in FILES}


def run(COMMAND, DIRECTORY, NAME, ENV=None):
    """Retain command/output even when a compiler or experiment fails."""
    write_json(DIRECTORY / (NAME + '-command.json'), COMMAND)
    with (DIRECTORY / (NAME + '.log')).open('w', encoding='utf-8') as LOG:
        RESULT = subprocess.run(COMMAND, cwd=DIRECTORY, stdout=LOG, stderr=subprocess.STDOUT, env=ENV)
    require(RESULT.returncode == 0, f'{NAME} failed; see {DIRECTORY / (NAME + ".log")}')


def prepare(OUTPUT):
    """Copy current headers and apply the explicit test-only patch with zero fuzz."""
    from generate import PINNED_HEADER, build_manifest
    from .prediction_cpp import render
    from .formatting import format_cpp
    from .verify.algebra import verify_identities

    TEXT = format_cpp(render())
    require((REPO_DIR / PINNED_HEADER).read_bytes() == TEXT.encode(), 'regenerate the pinned C++ header first')
    verify_identities(TEXT)
    MANIFEST = {'generator': build_manifest(REPO_DIR, TEXT), 'sources': evaluation_sources()}
    IDENTITY = hashlib.sha256(json.dumps(MANIFEST, sort_keys=True).encode()).hexdigest()
    SNAPSHOTS = OUTPUT / 'snapshots' / IDENTITY
    if not SNAPSHOTS.exists():
        for VARIANT in ('dense', 'generated'):
            DIRECTORY = SNAPSHOTS / VARIANT
            shutil.copytree(REPO_DIR / 'include', DIRECTORY / 'include')
        DIRECTORY = SNAPSHOTS / 'generated'
        run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(ROOT / 'tests/integration.patch')],
            DIRECTORY, 'patch')
        write_json(SNAPSHOTS / 'source.json', MANIFEST)
        write_json(SNAPSHOTS / 'headers.json', {
            str(FILE.relative_to(SNAPSHOTS)): digest(FILE)
            for VARIANT in ('dense', 'generated')
            for FILE in sorted((SNAPSHOTS / VARIANT / 'include').rglob('*.hpp'))
        })
    require(json.loads((SNAPSHOTS / 'source.json').read_text()) == MANIFEST, 'snapshot identity changed')
    for NAME, HASH in json.loads((SNAPSHOTS / 'headers.json').read_text()).items():
        require(digest(SNAPSHOTS / NAME) == HASH, f'modified experiment snapshot: {NAME}')
    return SNAPSHOTS


def backend_flags(ARGS, BACKEND):
    if BACKEND == 'eigen':
        require((ARGS.eigen / 'Eigen/Core').is_file(), 'Eigen headers not found; set --eigen')
        return ['-I' + str(ARGS.eigen), '-DEIGEN_RUNTIME_NO_MALLOC']
    require(ARGS.px4 is not None, 'PX4 backend requires --px4 checkout; it is never downloaded')
    require((ARGS.px4 / 'src/lib/matrix/matrix/math.hpp').is_file(), 'invalid PX4 checkout')
    return ['-DFORMAL_ESKF_TEST_PX4_MATRIX', '-D__PX4_POSIX',
            '-I' + str(ARGS.px4 / 'src/lib/matrix'),
            '-I' + str(ARGS.px4 / 'platforms/common/include'),
            '-I' + str(ARGS.px4 / 'build/px4_sitl_formal_eskf')]


def flags(ARGS, SNAPSHOTS, BACKEND, VARIANT, OPT, MODE):
    return [ARGS.cxx, '-std=c++20', '-' + OPT, '-fno-fast-math', '-ffp-contract=off',
            '-fno-exceptions', '-fno-rtti', '-Wall', '-Wextra', '-Werror',
            '-I' + str(SNAPSHOTS / VARIANT / 'include'), '-I' + str(REPO_DIR / 'tests'),
            '-DGENERATED=' + str(int(VARIANT == 'generated')), '-DESKF_QUAT_APPROX=' + str(MODE),
            '-DESKF_RESET_APPROX=0', '-DESKF_MAG_TILT=1'] + ARGS.cxx_flag + backend_flags(ARGS, BACKEND)


def dependencies(PATH):
    TEXT = PATH.read_text().replace('\\\n', ' ')
    return {NAME: digest(Path(NAME)) for NAME in sorted(set(shlex.split(TEXT.split(':', 1)[1])))}


def build_native(ARGS):
    SNAPSHOTS = prepare(ARGS.output)
    BUILDS = []
    for BACKEND, OPT, MODE, VARIANT in itertools.product(
            ARGS.backends, ARGS.optimizations, ARGS.modes, ('dense', 'generated')):
        NAME = f'{BACKEND}-{OPT}-q{MODE}-{VARIANT}'
        DIRECTORY = ARGS.output / 'native' / NAME
        DIRECTORY.mkdir(parents=True, exist_ok=True)
        COMMAND = flags(ARGS, SNAPSHOTS, BACKEND, VARIANT, OPT, MODE)
        if ARGS.sanitize:
            COMMAND += ['-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
        COMMAND += ['-MD', '-MF', str(DIRECTORY / 'dependencies.d'), str(NATIVE_DIR / 'native.cpp'),
                    '-o', str(DIRECTORY / 'run')]
        print('Build and check', NAME, flush=True)
        run(COMMAND, DIRECTORY, 'compile')
        ENV = {**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0:halt_on_error=1',
               'UBSAN_OPTIONS': 'halt_on_error=1:print_stacktrace=1'}
        run([str(DIRECTORY / 'run')], DIRECTORY, 'native', ENV)
        BUILDS.append({
            'backend': BACKEND, 'opt': OPT, 'mode': MODE, 'variant': VARIANT,
            'directory': str(DIRECTORY), 'command': COMMAND,
            'compiler': subprocess.check_output([ARGS.cxx, '--version'], text=True),
            'compiler_sha256': digest(Path(shutil.which(ARGS.cxx))),
            'binary_sha256': digest(DIRECTORY / 'run'), 'records_sha256': digest(DIRECTORY / 'native.log'),
            'dependencies': dependencies(DIRECTORY / 'dependencies.d'),
        })
    REPORT = {'snapshot': str(SNAPSHOTS), 'builds': BUILDS, 'sanitizers': ARGS.sanitize,
              'leak_check': 'not run', 'scope': 'test-only covariance integration, not the production core'}
    write_json(ARGS.output / 'builds.json', REPORT)
    return REPORT


def verify_builds(OUTPUT):
    REPORT = json.loads((OUTPUT / 'builds.json').read_text())
    SNAPSHOTS = Path(REPORT['snapshot'])
    STORED = json.loads((SNAPSHOTS / 'source.json').read_text())
    require(evaluation_sources() == STORED['sources'], 'stale evaluation source inventory')
    from generate import PINNED_HEADER, build_manifest
    require(STORED['generator'] == build_manifest(REPO_DIR, (REPO_DIR / PINNED_HEADER).read_text()),
            'stale generation/specification/formatter identity')
    for NAME, HASH in json.loads((SNAPSHOTS / 'headers.json').read_text()).items():
        require(digest(SNAPSHOTS / NAME) == HASH, f'stale snapshot: {NAME}')
    for BUILD in REPORT['builds']:
        DIRECTORY = Path(BUILD['directory'])
        require(digest(DIRECTORY / 'run') == BUILD['binary_sha256'], 'changed native binary')
        require(digest(DIRECTORY / 'native.log') == BUILD['records_sha256'], 'changed native records')
        for NAME, HASH in BUILD['dependencies'].items():
            require(digest(Path(NAME)) == HASH, f'changed compiled dependency: {NAME}')
    return REPORT


def benchmark(ARGS):
    (ARGS.output / 'performance.json').unlink(missing_ok=True)
    REPORT = verify_builds(ARGS.output)
    CERTIFICATE = json.loads((ARGS.output / 'numerical.json').read_text())
    require(CERTIFICATE['builds_sha256'] == digest(ARGS.output / 'builds.json'),
            'benchmark requires current numerical certification')
    require(not REPORT['sanitizers'], 'benchmark must use an uninstrumented build')
    BUILDS = [BUILD for BUILD in REPORT['builds'] if BUILD['mode'] == 0]
    require(BUILDS, 'benchmark requires default Exp-mode builds')
    CPU = min(os.sched_getaffinity(0))
    os.sched_setaffinity(0, {CPU})
    ROWS = []
    for ROUND in range(ARGS.rounds):
        for BUILD in (BUILDS if ROUND % 2 == 0 else list(reversed(BUILDS))):
            DIRECTORY = Path(BUILD['directory'])
            NAME = f'timing-{ROUND}'
            print('Measure', ROUND + 1, DIRECTORY.name, flush=True)
            run([str(DIRECTORY / 'run'), 'benchmark'], DIRECTORY, NAME)
            SAMPLES = [json.loads(LINE) for LINE in (DIRECTORY / (NAME + '.log')).read_text().splitlines()]
            require(len(SAMPLES) == 60, 'incomplete benchmark samples')
            ROWS += [{**SAMPLE, **{KEY: BUILD[KEY] for KEY in ('backend', 'variant', 'opt')}, 'round': ROUND}
                     for SAMPLE in SAMPLES]
    RESULTS = []
    for BACKEND, OPT in sorted({(BUILD['backend'], BUILD['opt']) for BUILD in BUILDS}):
        for BYTES, OP in itertools.product((4, 8), range(3)):
            SELECTED = [ROW for ROW in ROWS if (ROW['backend'], ROW['opt'], ROW['bytes'], ROW['op']) ==
                        (BACKEND, OPT, BYTES, OP)]
            MEDIANS = {VARIANT: [statistics.median(ROW['ns'] for ROW in SELECTED
                                                if ROW['variant'] == VARIANT and ROW['round'] == ROUND)
                                 for ROUND in range(ARGS.rounds)] for VARIANT in ('dense', 'generated')}
            RATIOS = [D / G for D, G in zip(MEDIANS['dense'], MEDIANS['generated'])]
            RESULTS.append({'backend': BACKEND, 'opt': OPT, 'bits': BYTES * 8,
                            'operation': ('covariance', 'predict', 'predict_accel_mag')[OP],
                            'dense_ns': statistics.median(MEDIANS['dense']),
                            'generated_ns': statistics.median(MEDIANS['generated']),
                            'speedup': statistics.median(RATIOS), 'paired_ratios': RATIOS})
    write_json(ARGS.output / 'performance.json', {'cpu': CPU, 'samples': ROWS, 'results': RESULTS,
                                               'claim': 'host observations, not target timing or WCET'})
