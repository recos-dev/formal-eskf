"""Independent product verifier regression and deliberate-corruption tests."""

from pathlib import Path
import sys
import unittest
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ins.verify.product_reference import contracts
from ins.verify.product_source import parse_header, verify
from ins.formatting import format_cpp
from ins.verify.product_numerical import check
from ins.verify import product_lean
from unittest.mock import patch
import copy


class ProductTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.TEXT = (ROOT.parent / 'include/formal_eskf/eskf/generated/ins_reset.hpp').read_text()

    def test_complete_generated_sources(self):
        COUNT = 0
        for GROUP in ('reset', 'innovation', 'joseph'):
            TEXT = (ROOT.parent / f'include/formal_eskf/eskf/generated/ins_{GROUP}.hpp').read_text()
            self.assertEqual(TEXT, format_cpp(TEXT))
            for NAME, PROGRAM in parse_header(TEXT, GROUP).items():
                COUNT += verify(PROGRAM, contracts()[NAME])
        self.assertEqual(COUNT, 1542)

    def test_structure_rejects_mutations(self):
        CHANGES = [
            ('return result;', 'return P;'),
            ('P(0U, 0U)', 'P(15U, 0U)'),
            ('result(1U, 0U) = result(0U, 1U);', 'result(1U, 0U) = result(0U, 2U);'),
            ('result(1U, 0U) = result(0U, 1U);', ''),
            ('#pragma once', '#pragma once\n#define injected 1'),
        ]
        for OLD, NEW in CHANGES:
            with self.subTest(change=OLD):
                self.assertIn(OLD, self.TEXT)
                with self.assertRaises((ValueError, SyntaxError)):
                    parse_header(self.TEXT.replace(OLD, NEW, 1), 'reset')

    def test_algebra_rejects_mutation(self):
        BAD = self.TEXT.replace('J(0U, 0U)', 'J(0U, 1U)', 1)
        PROGRAM = parse_header(BAD, 'reset')['ins_reset']
        with self.assertRaisesRegex(ValueError, 'identity mismatch'):
            verify(PROGRAM, contracts()['ins_reset'])

    def test_optimized_python(self):
        CODE = (
            "from pathlib import Path; from ins.verify.product_source import parse_header; "
            f"T=Path({str(ROOT.parent / 'include/formal_eskf/eskf/generated/ins_reset.hpp')!r}).read_text(); "
            "parse_header(T.replace('P(0U, 0U)','P(15U, 0U)',1),'reset')"
        )
        RESULT = subprocess.run(
            [sys.executable, '-O', '-c', CODE], cwd=ROOT, capture_output=True, text=True
        )
        self.assertNotEqual(RESULT.returncode, 0)
        self.assertIn('out-of-bounds', RESULT.stderr)

    def test_native_corruption_is_rejected(self):
        ONE = 0x3FF0000000000000  # IEEE binary64 1.0.
        P = [ONE if I == J else 0 for I in range(15) for J in range(15)]
        J = [ONE if I == K else 0 for I in range(3) for K in range(3)]
        RECORD = {
            'bits': 64,
            'P': P,
            'J': J,
            'output': P[:],
            'dense': P[:],
            'finite': True,
            'dense_finite': True,
        }
        SPEC = contracts()['ins_reset']
        PROGRAM = parse_header(self.TEXT, 'reset')['ins_reset']
        self.assertEqual(check(RECORD, PROGRAM, SPEC)['domain'], 'certified_finite')
        BAD = copy.deepcopy(RECORD)
        BAD['output'][0] ^= 1
        with self.assertRaisesRegex(ValueError, 'IEEE RNE'):
            check(BAD, PROGRAM, SPEC)
        BAD = copy.deepcopy(RECORD)
        BAD['P'][1] ^= 1
        with self.assertRaisesRegex(ValueError, 'asymmetric input'):
            check(BAD, PROGRAM, SPEC)

    def test_lean_export_matches_pinned_modules(self):
        TEMP = ROOT.parent / 'output/codegen'
        TEMP.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=TEMP) as DIRECTORY:
            DIRECTORY = Path(DIRECTORY)
            with patch.object(product_lean, 'DEST', DIRECTORY):
                product_lean.main()
            PINNED = ROOT.parent / 'formal/lean/FormalESKF/codegen/products'
            self.assertEqual(sorted(P.name for P in DIRECTORY.glob('*.lean')),
                             sorted(P.name for P in PINNED.glob('*.lean')))
            for FILE in DIRECTORY.glob('*.lean'):
                self.assertEqual(FILE.read_bytes(), (PINNED / FILE.name).read_bytes(), FILE.name)


if __name__ == '__main__':
    unittest.main()
