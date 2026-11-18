"""GCC frame/call-graph analysis and Linux observed stack measurements.

Unresolved external frames remain unknown; neither result is a target WCET or
whole-program stack bound. Cortex-M7 compilation is an optional separate profile.
"""

import itertools
from pathlib import Path
import re
import shutil
import subprocess
import json

from .evaluation import NATIVE_DIR, dependencies, digest, flags, prepare, run, write_json
from .verify import require

def graph(PATH):
    NODES = {}
    EDGES = {}
    for LINE in PATH.read_text().splitlines():
        MATCH = re.match(r'node: \{ title: "([^"]+)" label: "(.*)"', LINE)
        if MATCH:
            SIZE = re.search(r'\\n(\d+) bytes \(([^)]+)\)', MATCH[2])
            NODES[MATCH[1]] = {
                'label': MATCH[2].split(r'\n')[0],
                'bytes': int(SIZE[1]) if SIZE else None,
                'kind': SIZE[2] if SIZE else 'external',
            }
        MATCH = re.match(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', LINE)
        if MATCH:
            EDGES.setdefault(MATCH[1], []).append(MATCH[2])
    # Resolve constructor aliases using ELF section/value/size, never a name guess.
    SYMBOLS = {}
    LOCATIONS = {}
    for LINE in subprocess.check_output(['readelf', '-sW', str(PATH.with_suffix('.o'))], text=True).splitlines():
        FIELDS = LINE.split()
        if len(FIELDS) == 8 and FIELDS[0].endswith(':') and FIELDS[3] == 'FUNC' and FIELDS[6] != 'UND':
            LOCATION = tuple(FIELDS[I] for I in (1, 2, 6))
            SYMBOLS[FIELDS[7]] = LOCATION
            LOCATIONS.setdefault(LOCATION, []).append(FIELDS[7])
    NODE_SYMBOLS = {NAME.rsplit(':', 1)[-1]: NAME for NAME in NODES}
    ALIASES = {}
    for CHILD in {C for CHILDREN in EDGES.values() for C in CHILDREN} - NODES.keys():
        SYMBOL = CHILD.rsplit(':', 1)[-1]
        MATCHES = [NODE_SYMBOLS[S] for S in LOCATIONS.get(SYMBOLS.get(SYMBOL), []) if S in NODE_SYMBOLS]
        if MATCHES:
            ALIASES[CHILD] = MATCHES[0]

    def walk(NAME, SEEN):
        NAME = ALIASES.get(NAME, NAME)
        require(NAME not in SEEN, 'recursive stack graph')
        if NAME not in NODES:
            return 0, [], {'unresolved:' + NAME}
        NODE = NODES[NAME]
        if NODE['bytes'] is None:
            return 0, [], {NAME}
        UNKNOWN = set() if NODE['kind'] in ('static', 'dynamic,bounded') else {'dynamic-frame:' + NAME}
        CHILDREN = [walk(CHILD, SEEN | {NAME}) for CHILD in EDGES.get(NAME, [])]
        BEST = max(CHILDREN, key=lambda CHILD: CHILD[0], default=(0, [], set()))
        return (NODE['bytes'] + BEST[0], [{'symbol': NAME, **NODE}] + BEST[1],
                UNKNOWN.union(*(CHILD[2] for CHILD in CHILDREN)))

    RESULT = {}
    for ENTRY in ('call_kernel', 'call_predict'):
        SIZE, FRAMES, UNKNOWN = walk(ENTRY, set())
        RESULT[ENTRY] = {'known_frame_path_bytes': SIZE, 'maximum_known_path': FRAMES,
                         'external_or_dynamic_stack_unknown': sorted(UNKNOWN)}
    RESULT['elf_resolved_aliases'] = ALIASES
    return RESULT


def measure_stack(ARGS):
    (ARGS.output / 'stack.json').unlink(missing_ok=True)
    SNAPSHOTS = prepare(ARGS.output)
    RESULTS = []
    TARGETS = ['host'] + (['m7'] if ARGS.arm_cxx else [])
    for TARGET, BACKEND, SCALAR, VARIANT, OPT in itertools.product(
            TARGETS, ARGS.backends, ('float', 'double'), ('dense', 'generated'), ARGS.optimizations):
        NAME = '-'.join((TARGET, BACKEND, SCALAR, VARIANT, OPT))
        DIRECTORY = ARGS.output / 'stack' / NAME
        DIRECTORY.mkdir(parents=True, exist_ok=True)
        COMMAND = flags(ARGS, SNAPSHOTS, BACKEND, VARIANT, OPT, 0)
        if TARGET == 'm7':
            COMMAND[0] = ARGS.arm_cxx
            COMMAND = [ARG for ARG in COMMAND if ARG != '-D__PX4_POSIX']
            COMMAND += ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard']
        COMMAND += ['-DSCALAR=' + SCALAR, '-fstack-usage', '-fcallgraph-info=su', '-MD', '-MF',
                    str(DIRECTORY / 'dependencies.d'), '-c', str(NATIVE_DIR / 'stack.cpp'),
                    '-o', str(DIRECTORY / 'probe.o')]
        print('Stack', NAME, flush=True)
        RESULT = {'target': TARGET, 'backend': BACKEND, 'scalar': SCALAR, 'variant': VARIANT,
                  'opt': OPT, 'command': COMMAND, 'directory': str(DIRECTORY)}
        try:
            run(COMMAND, DIRECTORY, 'compile')
        except ValueError:
            if TARGET == 'host':
                raise
            RESULT['result'] = 'unavailable'
            RESULT['reason'] = (DIRECTORY / 'compile.log').read_text()
            RESULTS.append(RESULT)
            continue
        RESULT['result'] = 'measured'
        RESULT['dependencies'] = dependencies(DIRECTORY / 'dependencies.d')
        RESULT['object_sha256'] = digest(DIRECTORY / 'probe.o')
        RESULT['compiler'] = subprocess.check_output([COMMAND[0], '--version'], text=True)
        RESULT['compiler_sha256'] = digest(Path(shutil.which(COMMAND[0])))
        RESULT['stack'] = graph(DIRECTORY / 'probe.ci')
        RESULT['size'] = subprocess.check_output(['size', str(DIRECTORY / 'probe.o')], text=True)
        if TARGET == 'host':
            LINK = [ARGS.cxx, '-std=c++20', '-' + OPT, str(NATIVE_DIR / 'stack_host.cpp'),
                    str(DIRECTORY / 'probe.o'), '-o', str(DIRECTORY / 'probe')]
            run(LINK, DIRECTORY, 'link')
            run([str(DIRECTORY / 'probe')], DIRECTORY, 'observed')
            RESULT['observed_stack'] = [json.loads(LINE) for LINE in
                                        (DIRECTORY / 'observed.log').read_text().splitlines()]
        RESULTS.append(RESULT)
    write_json(ARGS.output / 'stack.json', {'snapshot': str(SNAPSHOTS), 'results': RESULTS,
                                         'claim': 'known frames and observed writes; external stack remains unknown'})
