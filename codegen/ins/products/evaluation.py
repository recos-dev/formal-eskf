#!/usr/bin/env python3
"""Sequential native verification, timing and static resource evidence."""

import argparse
import os
import json
from pathlib import Path
import shutil
import subprocess

from ins.evaluation import backend_flags, dependencies, digest, run, write_json
from ins.verify import require
from ins.verify.product_reference import contracts
from ins.resources import graph
from ins.formatting import format_cpp

ROOT = Path(__file__).resolve().parents[3]


def flags(ARGS, BACKEND, OPT=None):
    FLAGS = [
        ARGS.cxx,
        '-std=c++20',
        '-' + (OPT or ARGS.optimization),
        '-fno-fast-math',
        '-ffp-contract=off',
        '-fno-exceptions',
        '-fno-rtti',
        '-Wall',
        '-Wextra',
        '-Werror',
        '-I' + str(ROOT / 'include'),
    ]
    FLAGS += ARGS.cxx_flag + backend_flags(ARGS, BACKEND)
    if BACKEND == 'px4':
        FLAGS += ['-DUSE_PX4']
    return FLAGS


def identity(COMMAND, DIRECTORY, BINARY):
    return {
        'command': COMMAND,
        'dependencies': dependencies(DIRECTORY / 'dependencies.d'),
        'binary_sha256': digest(BINARY),
        'compiler_sha256': digest(Path(shutil.which(COMMAND[0]))),
        'compiler_version': subprocess.check_output([COMMAND[0], '--version'], text=True),
    }


def native(ARGS):
    OUTPUT = ARGS.output
    PROFILES = [(B, R, T, ARGS.optimization, ARGS.cxx, ARGS.sanitize)
                for B in ARGS.backends for R in ARGS.reset_modes for T in ARGS.tilt_modes]
    OLD = (
        json.loads((OUTPUT / 'native.json').read_text())['results']
        if (OUTPUT / 'native.json').exists()
        else []
    )
    OLD = {R['profile']: R for R in OLD}
    (OUTPUT / 'native.json').unlink(missing_ok=True)
    RESULTS = []
    for BACKEND, RESET, TILT, OPT, COMPILER, SAN in PROFILES:
        NAME = f'{BACKEND}-r{RESET}t{TILT}-{OPT}-{Path(COMPILER).name}' + ('-san' if SAN else '')
        DIRECTORY = OUTPUT / NAME
        DIRECTORY.mkdir(exist_ok=True)
        BINARY = DIRECTORY / 'native'
        COMMAND = flags(ARGS, BACKEND, OPT)
        COMMAND += [
            f'-DESKF_RESET_APPROX={RESET}',
            f'-DESKF_MAG_TILT={TILT}',
            '-MD',
            '-MF',
            str(DIRECTORY / 'dependencies.d'),
        ]
        if SAN:
            COMMAND += [
                '-fsanitize=address,undefined',
                '-fno-omit-frame-pointer',
                '-fno-sanitize-recover=all',
                '-fno-pie',
                '-no-pie',
            ]
        COMMAND += [str(ROOT / 'codegen/tests/native/products.cpp'), '-o', str(BINARY)]
        print('Native', NAME, flush=True)
        PREVIOUS = OLD.get(NAME, {})
        CURRENT = (
            PREVIOUS.get('command') == COMMAND
            and BINARY.exists()
            and PREVIOUS.get('binary_sha256') == digest(BINARY)
            and PREVIOUS.get('compiler_sha256') == digest(Path(shutil.which(COMPILER)))
            and all(
                Path(P).exists() and digest(Path(P)) == H
                for P, H in PREVIOUS.get('dependencies', {}).items()
            )
            and (DIRECTORY / 'records.log').exists()
            and PREVIOUS.get('records_sha256') == digest(DIRECTORY / 'records.log')
        )
        if not CURRENT:
            run(COMMAND, DIRECTORY, 'compile')
            run([str(BINARY)], DIRECTORY, 'records')
        run(
            [
                'python3',
                str(ROOT / 'codegen/products.py'),
                'certify',
                str(DIRECTORY / 'records.log'),
            ],
            DIRECTORY,
            'certify',
        )
        RESULTS.append(
            {
                'profile': NAME,
                **identity(COMMAND, DIRECTORY, BINARY),
                'records_sha256': digest(DIRECTORY / 'records.log'),
                'certificate_sha256': digest(DIRECTORY / 'records.certified.json'),
            }
        )
        write_json(
            OUTPUT / 'native.json',
            {
                'status': 'complete' if len(RESULTS) == len(PROFILES) else 'partial',
                'results': RESULTS,
            },
        )


def stack_source(OUTPUT):
    LINES = [
        '#define PRODUCT_KERNELS_ONLY',
        '#include "' + str(ROOT / 'codegen/tests/native/products.cpp') + '"',
        '#include <codegen_fixture.hpp>',
        'using S = SCALAR;',
    ]
    ENTRIES = []
    for NAME, SPEC in contracts().items():
        for N, (R, C) in SPEC['dimensions'].items():
            LINES.append(f'Matrix<S,{R}U,{C}U> {NAME}_{N};')
        R, C = SPEC['shape']
        LINES.append(f'Matrix<S,{R}U,{C}U> {NAME}_output;')
        CALL = ', '.join(NAME + '_' + N for N in SPEC['dimensions'])
        VARIANTS = ['dense', 'generated'] + (['block'] if SPEC['kind'] == 'reset' else [])
        for VARIANT in VARIANTS:
            ENTRY = VARIANT + '_' + NAME
            ENTRIES.append(ENTRY)
            LINES += [
                f'extern "C" __attribute__((noinline)) int call_{ENTRY}() {{',
                f'{NAME}_output = {VARIANT}_{NAME}<S>({CALL}); return 0;',
                '}',
            ]
    LINES += [
        'using State = formal_eskf::configuration::Ins::NominalState<Backend<S>>;',
        'State full_state, full_output_state;',
        'Matrix<S,15U,15U> full_P, full_output_P;',
        'Matrix<S,3U,3U> full_V;',
        'Matrix<S,3U,1U> full_reference, full_measurement;',
    ]
    ENTRIES.append('full_correct')
    LINES += [
        'extern "C" __attribute__((noinline)) int call_full_correct() {',
        'return formal_eskf::succeeded(formal_eskf::correct_magnetometer(',
        'full_state, full_P, full_measurement, full_reference, full_V, S{1e-6},',
        'full_output_state, full_output_P)) ? 0 : 1;',
        '}',
    ]
    LINES += [
        'extern "C" int setup(unsigned seed) {',
        '#ifndef USE_PX4',
        'Eigen::internal::set_is_malloc_allowed(false);',
        '#endif',
        'Random random{28374U+seed};',
    ]
    for NAME, SPEC in contracts().items():
        for N in SPEC['dimensions']:
            LINES.append(f'fill({NAME}_{N},random);')
    LINES += [
        'positive(full_P,random); positive(full_V,random);',
        'full_state = State{};',
        'full_reference(0U)=S{0.4}; full_reference(1U)=S{0}; full_reference(2U)=S{0.2};',
        'full_measurement = full_reference; full_measurement(0U) += S{0.01};',
        'return 0;',
        '}',
    ]
    PATH = OUTPUT / 'stack.cpp'
    PATH.write_text(format_cpp('\n'.join(LINES) + '\n'))
    return PATH, ['call_' + E for E in ENTRIES]


def stack_observer(OUTPUT, ENTRIES):
    TEXT = (ROOT / 'codegen/tests/native/stack_host.cpp').read_text()
    TEXT = TEXT.replace(
        'extern "C" int call_predict();\nextern "C" int call_kernel();',
        '\n'.join(f'extern "C" int {E}();' for E in ENTRIES)
        + '\nstatic int (*const operations[])() = {nullptr, '
        + ', '.join(ENTRIES)
        + '};',
    )
    TEXT = TEXT.replace(
        'operation == 0U ? 0 : (operation == 1U ? call_kernel() : call_predict())',
        'operation == 0U ? 0 : operations[operation]()',
    )
    TEXT = TEXT.replace('operation < 3U', f'operation < {len(ENTRIES)+1}U')
    TEXT = TEXT.replace(
        'if (call_predict() || call_kernel())', 'if (operation != 0U && operations[operation]())'
    )
    PATH = OUTPUT / 'stack_observer.cpp'
    PATH.write_text(format_cpp(TEXT))
    return PATH


def stack(ARGS):
    OUTPUT = ARGS.output
    from .callers import fixture

    from .callers import prepare

    prepare(OUTPUT / 'caller-include')
    SOURCE, ENTRIES = stack_source(OUTPUT)
    OBSERVER = stack_observer(OUTPUT, ENTRIES)
    (OUTPUT / 'stack.json').unlink(missing_ok=True)
    RESULTS = []
    for BACKEND, SCALAR, GENERATED in (
        (B, S, G) for B in ARGS.backends for S in ('float', 'double') for G in (0, 1)
    ):
        NAME = f'stack-{BACKEND}-{SCALAR}-g{GENERATED}'
        DIRECTORY = OUTPUT / NAME
        DIRECTORY.mkdir(exist_ok=True)
        (DIRECTORY / 'codegen_fixture.hpp').write_text(fixture(bool(GENERATED)))
        OBJECT = DIRECTORY / 'probe.o'
        COMMAND = flags(ARGS, BACKEND)
        COMMAND[1:1] = ['-I' + str(DIRECTORY), '-I' + str(OUTPUT / 'caller-include')]
        COMMAND += [
            '-DSCALAR=' + SCALAR,
            '-DESKF_RESET_APPROX=0',
            '-DESKF_MAG_TILT=1',
            '-fstack-usage',
            '-fcallgraph-info=su',
            '-MD',
            '-MF',
            str(DIRECTORY / 'dependencies.d'),
            '-c',
            str(SOURCE),
            '-o',
            str(OBJECT),
        ]
        print(NAME, flush=True)
        run(COMMAND, DIRECTORY, 'compile')
        run(
            [
                ARGS.cxx,
                '-std=c++20',
                '-O2',
                str(OBSERVER),
                str(OBJECT),
                '-o',
                str(DIRECTORY / 'observed'),
            ],
            DIRECTORY,
            'link',
        )
        run([str(DIRECTORY / 'observed')], DIRECTORY, 'observed')
        RESULTS.append(
            {
                'profile': NAME,
                'entries': ENTRIES,
                **identity(COMMAND, DIRECTORY, OBJECT),
                'stack': graph(DIRECTORY / 'probe.ci', ENTRIES),
                'observed': [
                    json.loads(L) for L in (DIRECTORY / 'observed.log').read_text().splitlines()
                ],
                'size': subprocess.check_output(
                    ['nm', '-S', '--size-sort', '-C', str(OBJECT)], text=True
                ),
            }
        )
    write_json(
        OUTPUT / 'stack.json',
        {
            'scope': 'Compiler-known call paths; unresolved external frames are not bounded.',
            'results': RESULTS,
        },
    )


def callers(ARGS):
    OUTPUT = ARGS.output
    from .callers import prepare

    prepare(OUTPUT / 'caller-include')
    (OUTPUT / 'callers.json').unlink(missing_ok=True)
    RESULTS = []
    PROFILES = [(B, R, T, ARGS.sanitize)
                for B in ARGS.backends for R in ARGS.reset_modes for T in ARGS.tilt_modes]
    for BACKEND, RESET, TILT, SAN in PROFILES:
        NAME = f'callers-{BACKEND}-r{RESET}t{TILT}' + ('-san' if SAN else '')
        DIRECTORY = OUTPUT / NAME
        DIRECTORY.mkdir(exist_ok=True)
        BINARY = DIRECTORY / 'native'
        COMMAND = flags(ARGS, BACKEND)
        if SAN:
            COMMAND += [
                '-fsanitize=address,undefined',
                '-fno-omit-frame-pointer',
                '-fno-sanitize-recover=all',
                '-fno-pie',
                '-no-pie',
            ]
        COMMAND[1:1] = ['-I' + str(OUTPUT / 'caller-include')]
        COMMAND += [
            f'-DESKF_RESET_APPROX={RESET}',
            f'-DESKF_MAG_TILT={TILT}',
            '-MD',
            '-MF',
            str(DIRECTORY / 'dependencies.d'),
            str(ROOT / 'codegen/tests/native/product_callers.cpp'),
            '-o',
            str(BINARY),
        ]
        print('Checked callers', NAME, flush=True)
        run(COMMAND, DIRECTORY, 'compile')
        run([str(BINARY)], DIRECTORY, 'records')
        RESULTS.append(
            {
                'profile': NAME,
                **identity(COMMAND, DIRECTORY, BINARY),
                'records_sha256': digest(DIRECTORY / 'records.log'),
                'results': [
                    json.loads(LINE)
                    for LINE in (DIRECTORY / 'records.log').read_text().splitlines()
                ],
            }
        )
        write_json(
            OUTPUT / 'callers.json',
            {
                'status': 'complete' if len(RESULTS) == len(PROFILES) else 'partial',
                'results': RESULTS,
            },
        )


def verify_binary(PROFILE, OUTPUT):
    BINARY = OUTPUT / PROFILE['profile'] / 'native'
    require(digest(BINARY) == PROFILE['binary_sha256'], 'benchmark binary changed')
    require(all(Path(P).is_file() and digest(Path(P)) == H
                for P, H in PROFILE['dependencies'].items()), 'benchmark sources changed; rebuild')


def benchmark(ARGS):
    OUTPUT = ARGS.output
    (OUTPUT / 'benchmark-profile.json').unlink(missing_ok=True)
    CPU = str(min(os.sched_getaffinity(0)))
    NATIVE = json.loads((OUTPUT / 'native.json').read_text())
    CALLERS = json.loads((OUTPUT / 'callers.json').read_text())
    require(NATIVE['status'] == CALLERS['status'] == 'complete',
            'benchmark requires complete native and caller results')
    for PROFILE in NATIVE['results']:
        if PROFILE['profile'].endswith('-san'):
            continue
        verify_binary(PROFILE, OUTPUT)
        print('Benchmark', PROFILE['profile'], flush=True)
        DIRECTORY = OUTPUT / PROFILE['profile']
        run(['taskset', '-c', CPU, str(DIRECTORY / 'native'), 'benchmark'], DIRECTORY, 'benchmark')
    for PROFILE in CALLERS['results']:
        if PROFILE['profile'].endswith('-san'):
            continue
        verify_binary(PROFILE, OUTPUT)
        print('Benchmark', PROFILE['profile'], flush=True)
        DIRECTORY = OUTPUT / PROFILE['profile']
        run(['taskset', '-c', CPU, str(DIRECTORY / 'native'), 'benchmark'], DIRECTORY, 'benchmark')
    for BACKEND in ARGS.backends:
        DIRECTORY = OUTPUT / f'reset-benchmark-{BACKEND}'
        DIRECTORY.mkdir(exist_ok=True)
        COMMAND = flags(ARGS, BACKEND) + [
            str(ROOT / 'codegen/tests/native/product_reset_benchmark.cpp'),
            '-o',
            str(DIRECTORY / 'native'),
        ]
        run(COMMAND, DIRECTORY, 'compile')
        run(['taskset', '-c', CPU, str(DIRECTORY / 'native')], DIRECTORY, 'benchmark')
    write_json(
        OUTPUT / 'benchmark-profile.json',
        {
            'cpu': CPU,
            'cpu_model': next(
                L.split(':', 1)[1].strip()
                for L in Path('/proc/cpuinfo').read_text().splitlines()
                if L.startswith('model name')
            ),
            'scope': 'Paired host measurements, no target WCET claim.',
        },
    )


def conditioning(ARGS):
    from .callers import prepare

    OUTPUT = ARGS.output
    prepare(OUTPUT / 'caller-include')
    (OUTPUT / 'conditioning.json').unlink(missing_ok=True)
    RESULTS = []
    for BACKEND in ARGS.backends:
        NAME = 'conditioning-' + BACKEND
        DIRECTORY = OUTPUT / NAME
        DIRECTORY.mkdir(exist_ok=True)
        BINARY = DIRECTORY / 'native'
        COMMAND = flags(ARGS, BACKEND)
        COMMAND[1:1] = ['-I' + str(OUTPUT / 'caller-include')]
        COMMAND += [
            '-DESKF_RESET_APPROX=0',
            '-DESKF_MAG_TILT=1',
            '-MD',
            '-MF',
            str(DIRECTORY / 'dependencies.d'),
            str(ROOT / 'codegen/tests/native/product_conditioning.cpp'),
            '-o',
            str(BINARY),
        ]
        print(NAME, flush=True)
        run(COMMAND, DIRECTORY, 'compile')
        run([str(BINARY)], DIRECTORY, 'records')
        RESULTS.append(
            {
                'profile': NAME,
                **identity(COMMAND, DIRECTORY, BINARY),
                'records_sha256': digest(DIRECTORY / 'records.log'),
                'results': [
                    json.loads(L) for L in (DIRECTORY / 'records.log').read_text().splitlines()
                ],
            }
        )
        write_json(
            OUTPUT / 'conditioning.json',
            {'status': 'complete' if len(RESULTS) == len(ARGS.backends) else 'partial', 'results': RESULTS},
        )


def regressions(ARGS):
    OUTPUT = ARGS.output
    from .callers import prepare

    prepare(OUTPUT / 'caller-include')
    (OUTPUT / 'regressions.json').unlink(missing_ok=True)
    RESULTS = []
    for BACKEND in ARGS.backends:
        for RESET in ARGS.reset_modes:
            for TEST in ('test_eskf_injection', 'test_eskf_correction'):
                NAME = f'regression-{BACKEND}-r{RESET}-{TEST}'
                DIRECTORY = OUTPUT / NAME
                DIRECTORY.mkdir(exist_ok=True)
                BINARY = DIRECTORY / 'test'
                COMMAND = flags(ARGS, BACKEND)
                COMMAND[1:1] = ['-I' + str(OUTPUT / 'caller-include')]
                COMMAND += [
                    f'-DESKF_RESET_APPROX={RESET}',
                    '-MD',
                    '-MF',
                    str(DIRECTORY / 'dependencies.d'),
                    str(ROOT / 'tests' / (TEST + '.cpp')),
                    '-o',
                    str(BINARY),
                ]
                print(NAME, flush=True)
                run(COMMAND, DIRECTORY, 'compile')
                run([str(BINARY)], DIRECTORY, 'test')
                RESULTS.append(
                    {
                        'profile': NAME,
                        **identity(COMMAND, DIRECTORY, BINARY),
                        'log_sha256': digest(DIRECTORY / 'test.log'),
                    }
                )
                write_json(
                    OUTPUT / 'regressions.json',
                    {'status': 'complete' if len(RESULTS) == len(ARGS.backends) * len(ARGS.reset_modes) * 2 else 'partial', 'results': RESULTS},
                )


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument(
        'stage', choices=['native', 'stack', 'callers', 'benchmark', 'regressions', 'conditioning']
    )
    PARSER.add_argument('--output', type=Path, default=ROOT / 'output/codegen/products')
    PARSER.add_argument('--backends', nargs='+', choices=['eigen', 'px4'], default=['eigen'])
    PARSER.add_argument('--px4', type=Path, help='external PX4 checkout; never downloaded')
    PARSER.add_argument('--eigen', type=Path, default=Path('/usr/include/eigen3'))
    PARSER.add_argument('--cxx', default='g++')
    PARSER.add_argument('--cxx-flag', action='append', default=[])
    PARSER.add_argument('--optimization', choices=['O1', 'O2', 'O3'], default='O2')
    PARSER.add_argument('--reset-modes', nargs='+', type=int, choices=[0, 1], default=[0, 1])
    PARSER.add_argument('--tilt-modes', nargs='+', type=int, choices=[0, 1], default=[0, 1])
    PARSER.add_argument('--sanitize', action='store_true', help='native and callers stages only')
    ARGS = PARSER.parse_args()
    require(not ARGS.sanitize or ARGS.stage in ('native', 'callers'),
            '--sanitize applies only to native/callers')
    ARGS.output = ARGS.output.resolve()
    ARGS.eigen = ARGS.eigen.resolve()
    if ARGS.px4:
        ARGS.px4 = ARGS.px4.resolve()
    ARGS.output.mkdir(parents=True, exist_ok=True)
    STAGES = {'native': native, 'stack': stack, 'callers': callers, 'benchmark': benchmark,
              'regressions': regressions, 'conditioning': conditioning}
    STAGES[ARGS.stage](ARGS)


if __name__ == '__main__':
    main()
