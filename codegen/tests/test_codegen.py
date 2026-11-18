"""Failure controls for source parsing, numerical checks and artifact provenance."""

import copy
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from generate import SPEC_FILES, build_manifest, main, read_verified_header, tool_fingerprints
from ins.covariance import render
from ins.formatting import format_cpp
from ins.verify.algebra import verify_identities
from ins.verify.cpp import parse
from ins.verify.evidence import read_records
from ins.verify.lean import export
from ins.verify.numerical import check_record, evaluate_expression, power_of_two, round_ieee


class SourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.TEXT = render()

    def test_formatter_preserves_arithmetic(self):
        FORMATTED = format_cpp(self.TEXT)
        self.assertNotEqual(FORMATTED, self.TEXT)
        self.assertEqual(parse(FORMATTED), parse(self.TEXT))
        self.assertEqual(FORMATTED, format_cpp(FORMATTED))
        for OLD, NEW in [('dt =', 'd t ='), ('F(0U, 3U)', 'F(0U, /*hidden*/ 3U)')]:
            with self.assertRaises((ValueError, SyntaxError)):
                parse(FORMATTED.replace(OLD, NEW, 1))

    def test_whole_file_and_data_flow(self):
        CHANGES = [
            ('return result;', 'return P;'),
            ('#pragma once', '#pragma once\n#define private public'),
            ('    // END GENERATED DAG', '    result(0U, 0U) = value_type{0};\n    // END GENERATED DAG'),
            ('result(0U, 0U) =', 'result(0U, 15U) ='),
            ('value_type const dt = F(0U, 3U);', 'value_type const dt = undefined_name;'),
            ('value_type const dt = F(0U, 3U);', 'value_type const dt = abs(F(0U, 3U));'),
            ('F(0U, 3U)', 'F(0U0U, 3U)'),
            ('F(0U, 3U)', '--F(0U, 3U)'),
            ('F(0U, 3U)', '1U - F(0U, 3U)'),
            ('result(1U, 0U) = result(0U, 1U);', 'result(1U, 0U) = result(0U, 2U);'),
            ('    result(1U, 0U) = result(0U, 1U);\n', ''),
            ('    // END GENERATED DAG', '    value_type const c999 = value_type{0};\n    // END GENERATED DAG'),
        ]
        for OLD, NEW in CHANGES:
            with self.subTest(change=OLD):
                self.assertIn(OLD, self.TEXT)
                with self.assertRaises((ValueError, SyntaxError)):
                    parse(self.TEXT.replace(OLD, NEW, 1))
        with self.assertRaises(ValueError):
            parse(self.TEXT + '// ignored tail\n')

    def test_mirror_requires_complete_upper_triangle(self):
        LINES = self.TEXT.splitlines(keepends=True)
        FIRST_MIRROR = LINES.index('    result(1U, 0U) = result(0U, 1U);\n')
        LAST_UPPER = next(INDEX for INDEX, LINE in enumerate(LINES) if LINE.startswith('    result(14U, 14U) ='))
        # All upper rows except the last already exist, including this mirror's input.
        LINES[LAST_UPPER], LINES[FIRST_MIRROR] = LINES[FIRST_MIRROR], LINES[LAST_UPPER]
        with self.assertRaisesRegex(ValueError, 'mirror before all upper'):
            parse(''.join(LINES))

    def test_algebra_mutations(self):
        CHANGES = [
            ('F(3U, 6U)', 'F(3U, 7U)'),
            ('Q_i(0U, 0U)', 'Q_i(0U, 1U)'),
            ('F(0U, 3U)', '(-F(0U, 3U))'),
        ]
        for OLD, NEW in CHANGES:
            with self.subTest(change=OLD), self.assertRaisesRegex(ValueError, 'polynomial mismatch'):
                verify_identities(self.TEXT.replace(OLD, NEW, 1))

    def test_optimized_python_keeps_checks(self):
        CODE = (
            "from ins.verify.cpp import parse; from ins.covariance import render; "
            "parse(render().replace('return result;', 'return P;'))"
        )
        RESULT = subprocess.run([sys.executable, '-O', '-c', CODE], cwd=ROOT, capture_output=True)
        self.assertNotEqual(RESULT.returncode, 0)
        self.assertIn(b'unexpected function/header envelope', RESULT.stderr)


class NumericalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.PROGRAM = parse(render())

    @staticmethod
    def zero_record(BITS=32):
        WORD_FORMAT, FLOAT_FORMAT = ('<I', '<f') if BITS == 32 else ('<Q', '<d')
        ONE = struct.unpack(WORD_FORMAT, struct.pack(FLOAT_FORMAT, 1.0))[0]
        RECORD = {
            'bits': BITS, 'F': [0] * 225, 'P': [0] * 225,
            'R': [0] * 9, 'Q_i': [0] * 144, 'output': [0] * 225,
        }
        for I in range(15):
            RECORD['F'][16 * I] = ONE
        return RECORD

    def test_rounding_boundaries(self):
        for BITS, PRECISION, MIN_EXPONENT in [(32, 24, -149), (64, 53, -1074)]:
            with self.subTest(bits=BITS):
                ETA = power_of_two(MIN_EXPONENT)
                self.assertEqual(round_ieee(ETA / 2, BITS), 0)
                self.assertEqual(round_ieee(3 * ETA / 2, BITS), 2 * ETA)
                self.assertEqual(round_ieee(-3 * ETA / 2, BITS), -2 * ETA)
                self.assertEqual(round_ieee(1 + power_of_two(-PRECISION), BITS), 1)
                with self.assertRaises(OverflowError):
                    round_ieee(power_of_two(128 if BITS == 32 else 1024), BITS)
        with self.assertRaises(ValueError):
            round_ieee(power_of_two(0), 16)

    def test_records_and_numeric_failures(self):
        for BITS in (32, 64):
            RECORD = self.zero_record(BITS)
            check_record(RECORD, self.PROGRAM)
            for KEY, VALUE in [('bits', 16), ('F', []), ('Q_i', [0] * 143)]:
                BAD = copy.deepcopy(RECORD)
                BAD[KEY] = VALUE
                with self.assertRaises(ValueError):
                    check_record(BAD, self.PROGRAM)
            MUTATIONS = [
                ('F', 1, RECORD['F'][0]),
                ('Q_i', 1, RECORD['F'][0]),
                ('P', 1, RECORD['F'][0]),
                ('output', 0, RECORD['F'][0]),
                ('R', 0, -1),
                ('R', 0, 1 << BITS),
                ('R', 0, True),
                ('R', 0, 0x7f800000 if BITS == 32 else 0x7ff0000000000000),
            ]
            for KEY, INDEX, VALUE in MUTATIONS:
                BAD = copy.deepcopy(RECORD)
                BAD[KEY][INDEX] = VALUE
                with self.subTest(key=KEY, index=INDEX, value=VALUE), self.assertRaises(ValueError):
                    check_record(BAD, self.PROGRAM)
            DENSE = copy.deepcopy(RECORD)
            DENSE['R'][0] = RECORD['F'][0]
            with self.assertRaisesRegex(ValueError, 'different producer'):
                check_record(RECORD, self.PROGRAM, DENSE)

    def test_local_witnesses_are_checked(self):
        NODE = ('add', ('int', 1), ('int', 1))
        with patch('ins.verify.numerical.round_bound_up', return_value=-1):
            with self.assertRaisesRegex(ValueError, 'not upward'):
                evaluate_expression(NODE, {}, 32)
        with patch('ins.verify.numerical.round_ieee', return_value=3):
            with self.assertRaisesRegex(ValueError, 'local IEEE'):
                evaluate_expression(NODE, {}, 32)

    def test_optimized_python_keeps_record_checks(self):
        CODE = "from ins.verify.numerical import validate_record; validate_record({'bits': 16})"
        RESULT = subprocess.run([sys.executable, '-O', '-c', CODE], cwd=ROOT, capture_output=True)
        self.assertNotEqual(RESULT.returncode, 0)
        self.assertIn(b'invalid scalar format', RESULT.stderr)


class GenerationTests(unittest.TestCase):
    def setUp(self):
        TEMP = tempfile.TemporaryDirectory()
        self.addCleanup(TEMP.cleanup)
        self.CORE = Path(TEMP.name) / 'core'
        self.OUTPUT = Path(TEMP.name) / 'output'
        for NAME in ('include/formal_eskf/example.hpp', *SPEC_FILES):
            FILE = self.CORE / NAME
            FILE.parent.mkdir(parents=True, exist_ok=True)
            FILE.write_text('fixture\n', encoding='utf-8')

    def write_fixture(self, TEXT):
        self.OUTPUT.mkdir()
        (self.OUTPUT / 'ins_covariance.hpp').write_bytes(TEXT.encode('utf-8'))
        (self.OUTPUT / 'manifest.json').write_text(
            json.dumps(build_manifest(self.CORE, TEXT)), encoding='utf-8'
        )

    def test_stale_manifest_and_determinism(self):
        TEXT = format_cpp(render())
        self.assertEqual(TEXT, format_cpp(render()))
        self.write_fixture(TEXT)
        self.assertEqual(TEXT, read_verified_header(self.CORE, self.OUTPUT))
        (self.CORE / SPEC_FILES[0]).write_text('changed specification\n', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'stale'):
            read_verified_header(self.CORE, self.OUTPUT)
        (self.CORE / SPEC_FILES[0]).write_text('fixture\n', encoding='utf-8')
        (self.OUTPUT / 'ins_covariance.hpp').write_text(
            TEXT.replace('return result;', 'return P;'), encoding='utf-8'
        )
        with self.assertRaisesRegex(ValueError, 'stale'):
            read_verified_header(self.CORE, self.OUTPUT)

    def test_verifier_fingerprints_invalidate_manifest(self):
        self.write_fixture(render())
        SOURCES = tool_fingerprints()
        EXPECTED = {'generate.py', 'requirements.txt'}
        EXPECTED.update(str(FILE.relative_to(ROOT)) for FILE in (ROOT / 'ins').rglob('*.py'))
        self.assertEqual(EXPECTED, set(SOURCES))
        SOURCES['ins/verify/cpp.py'] = 'changed checker'
        with patch('generate.tool_fingerprints', return_value=SOURCES):
            with self.assertRaisesRegex(ValueError, 'stale'):
                read_verified_header(self.CORE, self.OUTPUT)

    def test_header_hash_checks_actual_bytes(self):
        TEXT = render()
        self.write_fixture(TEXT)
        (self.OUTPUT / 'ins_covariance.hpp').write_bytes(TEXT.replace('\n', '\r\n').encode('utf-8'))
        with self.assertRaisesRegex(ValueError, 'stale'):
            read_verified_header(self.CORE, self.OUTPUT)

    def test_invalid_generation_does_not_publish(self):
        ARGS = ['generate.py', '--core', str(self.CORE), '--output', str(self.OUTPUT)]
        INVALID = render().replace('F(3U, 6U)', 'F(3U, 7U)', 1)
        with patch.object(sys, 'argv', ARGS), patch('generate.render', return_value=INVALID):
            with self.assertRaisesRegex(ValueError, 'polynomial mismatch'):
                main()
        self.assertFalse(self.OUTPUT.exists())

    def test_cli_from_another_directory(self):
        ARGS = [
            sys.executable, str(ROOT / 'generate.py'),
            '--core', str(self.CORE), '--output', str(self.OUTPUT),
        ]
        GENERATED = subprocess.run(ARGS, cwd=self.CORE, capture_output=True, text=True)
        self.assertEqual(GENERATED.returncode, 0, GENERATED.stderr)
        BEFORE = {FILE.name: FILE.read_bytes() for FILE in self.OUTPUT.iterdir()}
        CHECKED = subprocess.run(ARGS + ['--check'], cwd=self.CORE, capture_output=True, text=True)
        self.assertEqual(CHECKED.returncode, 0, CHECKED.stderr)
        self.assertIn('225/225', CHECKED.stdout)
        self.assertEqual(BEFORE, {FILE.name: FILE.read_bytes() for FILE in self.OUTPUT.iterdir()})

    def test_lean_certificates_are_current(self):
        EXPECTED = ROOT.parent / 'formal/lean/FormalESKF/codegen'
        self.OUTPUT.mkdir()
        for NAME in ('Properties.lean', 'ErrorBounds.lean'):
            (self.OUTPUT / NAME).write_bytes((EXPECTED / NAME).read_bytes())
        export(format_cpp(render()), self.OUTPUT)
        self.assertEqual(
            {FILE.name for FILE in EXPECTED.glob('*.lean')},
            {FILE.name for FILE in self.OUTPUT.glob('*.lean')},
        )
        for FILE in self.OUTPUT.glob('*.lean'):
            with self.subTest(file=FILE.name):
                self.assertEqual(FILE.read_bytes(), (EXPECTED / FILE.name).read_bytes())

    def test_native_inventory_rejects_missing_evidence(self):
        ROWS = []
        for BITS in (32, 64):
            ROWS += [{'kind': 'status', 'bits': BITS, 'seed': SEED,
                      'covariance': int(SEED >= 64), 'predict': int(SEED >= 64)}
                     for SEED in range(728)]
            ROWS += [{'kind': 'numerical', 'bits': BITS, 'seed': SEED} for SEED in range(64)]
            ROWS.append({'kind': 'contracts', 'bits': BITS, 'calls': 5824, 'failures': 0})
        PATH = self.CORE / 'records.jsonl'
        PATH.write_text('\n'.join(json.dumps(ROW) for ROW in ROWS))
        read_records(PATH)
        for BAD in (ROWS[:-1], ROWS + ROWS[:1], [dict(ROWS[0], covariance=1)] + ROWS[1:]):
            PATH.write_text('\n'.join(json.dumps(ROW) for ROW in BAD))
            with self.assertRaises((ValueError, KeyError)):
                read_records(PATH)


if __name__ == '__main__':
    unittest.main()
