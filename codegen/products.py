#!/usr/bin/env python3
"""Generate and verify the standalone INS matrix products.

Commands: generate, derive, native-export, lean-export, esbmc-export, certify,
native, callers, stack, benchmark, controls, conditioning, regressions.
"""

import importlib
import sys
import subprocess

COMMANDS = {
    'generate': 'ins.products.generate',
    'derive': 'ins.products.derive',
    'native-export': 'ins.products.native',
    'lean-export': 'ins.verify.product_lean',
    'esbmc-export': 'ins.products.esbmc',
    'certify': 'ins.verify.product_numerical',
    'controls': 'ins.products.controls',
    'native': 'ins.products.evaluation',
    'callers': 'ins.products.evaluation',
    'stack': 'ins.products.evaluation',
    'benchmark': 'ins.products.evaluation',
    'regressions': 'ins.products.evaluation',
    'conditioning': 'ins.products.evaluation',
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        return
    ACTION = sys.argv.pop(1)
    if ACTION not in COMMANDS:
        raise SystemExit('Unknown command: ' + ACTION)
    if COMMANDS[ACTION] == 'ins.products.evaluation':
        sys.argv.insert(1, ACTION)
    importlib.import_module(COMMANDS[ACTION]).main()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError) as ERROR:
        sys.exit(f'error: {ERROR}')
