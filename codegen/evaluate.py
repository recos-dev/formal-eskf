#!/usr/bin/env python3
"""Reproduce native, numerical, benchmark and stack checks for the pinned INS kernel."""

import argparse
from pathlib import Path
import subprocess
import sys

from ins.evaluation import REPO_DIR, benchmark, build_native
from ins.verify.evidence import certify


def parse_args():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('stage', choices=('native', 'certify', 'benchmark', 'stack'))
    PARSER.add_argument('--output', type=Path, default=REPO_DIR / 'output/codegen/evaluation')
    PARSER.add_argument('--backends', nargs='+', choices=('eigen', 'px4'), default=['eigen'])
    PARSER.add_argument('--px4', type=Path, help='external PX4 checkout for the optional matrix backend')
    PARSER.add_argument('--eigen', type=Path, default=Path('/usr/include/eigen3'))
    PARSER.add_argument('--cxx', default='g++', help='host compiler; stack analysis requires GCC')
    PARSER.add_argument('--cxx-flag', action='append', default=[], help='extra host toolchain flag (repeatable)')
    PARSER.add_argument('--optimizations', nargs='+', choices=('O1', 'O2', 'O3'), default=['O2', 'O3'])
    PARSER.add_argument('--modes', nargs='+', type=int, choices=(0, 1), default=[0, 1])
    PARSER.add_argument('--sanitize', action='store_true', help='native ASan/UBSan; excludes leak checking')
    PARSER.add_argument('--rounds', type=int, default=5, help='alternating benchmark rounds')
    PARSER.add_argument('--arm-cxx', help='optional arm-none-eabi-g++ for Cortex-M7 stack/object checks')
    ARGS = PARSER.parse_args()
    if ARGS.rounds < 1:
        PARSER.error('--rounds must be positive')
    ARGS.output = ARGS.output.resolve()
    ARGS.eigen = ARGS.eigen.resolve()
    if ARGS.px4:
        ARGS.px4 = ARGS.px4.resolve()
    return ARGS


def main():
    ARGS = parse_args()
    ARGS.output.mkdir(parents=True, exist_ok=True)
    if ARGS.stage == 'native':
        # Invalidate the old completion marker before attempting any fresh build.
        (ARGS.output / 'builds.json').unlink(missing_ok=True)
        (ARGS.output / 'numerical.json').unlink(missing_ok=True)
        build_native(ARGS)
        certify(ARGS.output)
    elif ARGS.stage == 'certify':
        certify(ARGS.output)
    elif ARGS.stage == 'benchmark':
        benchmark(ARGS)
    else:
        from ins.resources import measure_stack
        measure_stack(ARGS)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as ERROR:
        sys.exit(f'error: {ERROR}')
