"""Exact-rational reference, IEEE value simulation and conservative error bounds.

Each evaluated expression carries (exact value, absolute error bound, rounded
value). Rational simulation does not preserve signed zero; stored mirror bits
are checked separately. Inputs must describe finite binary32/binary64 values.
"""

from fractions import Fraction
import struct

from . import require


def validate_record(RECORD):
    """Check the encoded matrices and the structured F / diagonal Q_i domain."""
    require(
        type(RECORD.get('bits')) is int and RECORD['bits'] in (32, 64),
        'invalid scalar format',
    )
    BITS = RECORD['bits']
    FRACTION_BITS, EXPONENT_BITS = (23, 8) if BITS == 32 else (52, 11)
    EXPONENT_MASK = (1 << EXPONENT_BITS) - 1
    for NAME, SIZE in [('F', 15), ('P', 15), ('R', 3), ('Q_i', 12), ('output', 15)]:
        WORDS = RECORD.get(NAME)
        require(isinstance(WORDS, list) and len(WORDS) == SIZE * SIZE, f'invalid {NAME} dimensions')
        for WORD in WORDS:
            require(type(WORD) is int and 0 <= WORD < (1 << BITS), 'invalid encoded coefficient')
            require(
                ((WORD >> FRACTION_BITS) & EXPONENT_MASK) != EXPONENT_MASK,
                'nonfinite encoded coefficient',
            )

    F = [decode_coefficient(WORD, BITS) for WORD in RECORD['F']]
    DT = F[3]
    for I in range(15):
        for J in range(15):
            if (3 <= I < 6 and 6 <= J < 12) or (6 <= I < 9 and 6 <= J < 9):
                continue
            EXPECTED = Fraction(1) if I == J else Fraction(0)
            if I < 3 and J == I + 3:
                EXPECTED = DT
            if 6 <= I < 9 and J == I + 6:
                EXPECTED = -DT
            require(F[15 * I + J] == EXPECTED, 'F violates the kernel block contract')
    for I in range(12):
        for J in range(12):
            if I != J:
                require(
                    decode_coefficient(RECORD['Q_i'][12 * I + J], BITS) == 0,
                    'Q_i must be diagonal',
                )


def power_of_two(EXPONENT):
    if EXPONENT >= 0:
        return Fraction(1 << EXPONENT)
    return Fraction(1, 1 << -EXPONENT)


def binary_exponent(VALUE):
    """Return floor(log2(VALUE)) for a positive rational value."""
    EXPONENT = VALUE.numerator.bit_length() - VALUE.denominator.bit_length()
    if VALUE < power_of_two(EXPONENT):
        EXPONENT -= 1
    return EXPONENT


def round_bound_up(VALUE):
    """Round a nonnegative bound upward to 192 significant binary digits."""
    if not VALUE:
        return VALUE
    STEP = power_of_two(binary_exponent(VALUE) - 191)
    SCALED = VALUE / STEP
    return ((SCALED.numerator + SCALED.denominator - 1) // SCALED.denominator) * STEP


def round_ieee(VALUE, BITS):
    """Simulate nearest ties-to-even with gradual underflow, rejecting overflow."""
    require(BITS in (32, 64), 'unsupported scalar format')
    if not VALUE:
        return Fraction(0)
    PRECISION, MIN_EXPONENT, MAX_EXPONENT = (24, -126, 127) if BITS == 32 else (53, -1022, 1023)
    SIGN = -1 if VALUE < 0 else 1
    MAGNITUDE = abs(VALUE)
    STEP = power_of_two(max(binary_exponent(MAGNITUDE), MIN_EXPONENT) - PRECISION + 1)
    SCALED = MAGNITUDE / STEP
    INTEGER, REMAINDER = divmod(SCALED.numerator, SCALED.denominator)
    ABOVE_HALF = 2 * REMAINDER > SCALED.denominator
    ODD_TIE = 2 * REMAINDER == SCALED.denominator and INTEGER % 2
    INTEGER += ABOVE_HALF or ODD_TIE
    ROUNDED = INTEGER * STEP
    MAX_FINITE = (2 - power_of_two(1 - PRECISION)) * power_of_two(MAX_EXPONENT)
    if ROUNDED > MAX_FINITE:
        raise OverflowError('DAG simulation overflow')
    return SIGN * ROUNDED


def evaluate_expression(NODE, VALUES, BITS):
    """Propagate exact values, bounds and rounded values in parsed source order."""
    OPERATOR = NODE[0]
    if OPERATOR == 'ref':
        return VALUES[NODE[1]]
    if OPERATOR == 'int':
        return Fraction(NODE[1]), Fraction(0), Fraction(NODE[1])
    if OPERATOR == 'neg':
        EXACT, BOUND, ROUNDED = evaluate_expression(NODE[1], VALUES, BITS)
        return -EXACT, BOUND, -ROUNDED

    EXACT_LEFT, BOUND_LEFT, ROUNDED_LEFT = evaluate_expression(NODE[1], VALUES, BITS)
    EXACT_RIGHT, BOUND_RIGHT, ROUNDED_RIGHT = evaluate_expression(NODE[2], VALUES, BITS)
    if OPERATOR == 'mul':
        EXACT = EXACT_LEFT * EXACT_RIGHT
        PROPAGATED = (
            abs(EXACT_LEFT) * BOUND_RIGHT
            + abs(EXACT_RIGHT) * BOUND_LEFT
            + BOUND_LEFT * BOUND_RIGHT
        )
        UNROUNDED = ROUNDED_LEFT * ROUNDED_RIGHT
    elif OPERATOR == 'add':
        EXACT = EXACT_LEFT + EXACT_RIGHT
        PROPAGATED = BOUND_LEFT + BOUND_RIGHT
        UNROUNDED = ROUNDED_LEFT + ROUNDED_RIGHT
    elif OPERATOR == 'sub':
        EXACT = EXACT_LEFT - EXACT_RIGHT
        PROPAGATED = BOUND_LEFT + BOUND_RIGHT
        UNROUNDED = ROUNDED_LEFT - ROUNDED_RIGHT
    else:
        raise ValueError(f'unsupported operator: {OPERATOR}')

    U, ETA = (
        (power_of_two(-24), power_of_two(-149))
        if BITS == 32 else (power_of_two(-53), power_of_two(-1074))
    )
    RAW_BOUND = (1 + U) * PROPAGATED + U * abs(EXACT) + ETA / 2
    BOUND = round_bound_up(RAW_BOUND)
    ROUNDED = round_ieee(UNROUNDED, BITS)
    require(BOUND >= RAW_BOUND, 'bound rounding was not upward')
    require(
        abs(ROUNDED - UNROUNDED) <= U * abs(UNROUNDED) + ETA / 2,
        'local IEEE rounding witness failed',
    )
    return EXACT, BOUND, ROUNDED


def decode_coefficient(WORD, BITS):
    FLOAT_FORMAT, WORD_FORMAT = ('<f', '<I') if BITS == 32 else ('<d', '<Q')
    VALUE = struct.unpack(FLOAT_FORMAT, struct.pack(WORD_FORMAT, WORD))[0]
    return Fraction(VALUE)


def reference_covariance(F, P, R, QI):
    """Evaluate the dense matrix formula on exact decoded producer values."""
    W = [
        [sum((F[I][K] * P[K][J] for K in range(15) if F[I][K]), Fraction(0)) for J in range(15)]
        for I in range(15)
    ]
    RESULT = []
    for I in range(15):
        ROW = []
        for J in range(15):
            VALUE = sum((W[I][K] * F[J][K] for K in range(15) if F[J][K]), Fraction(0))
            if 3 <= I < 6 and 3 <= J < 6:
                VALUE += sum(R[I - 3][K] * QI[K][K] * R[J - 3][K] for K in range(3))
            elif I == J and I >= 6:
                VALUE += QI[I - 3][I - 3]
            ROW.append(VALUE)
        RESULT.append(ROW)
    return RESULT


def check_record(RECORD, PROGRAM, DENSE=None):
    """Validate one native result against a parsed program and rational oracle.

    Optional DENSE output must use identical producer inputs. Its observed error
    is reported separately; the propagated bound applies to generated code only.
    """
    validate_record(RECORD)
    if DENSE is not None:
        validate_record(DENSE)
        require(
            all(RECORD[KEY] == DENSE[KEY] for KEY in ('bits', 'F', 'P', 'R', 'Q_i')),
            'dense comparison has different producer inputs',
        )
    BITS = RECORD['bits']
    VALUES = {}
    MATRICES = {}
    for NAME, SIZE in [('F', 15), ('P', 15), ('R', 3), ('Q_i', 12)]:
        MATRIX = [
            [decode_coefficient(RECORD[NAME][I * SIZE + J], BITS) for J in range(SIZE)]
            for I in range(SIZE)
        ]
        MATRICES[NAME] = MATRIX
        for I in range(SIZE):
            for J in range(SIZE):
                VALUES[NAME, I, J] = (MATRIX[I][J], Fraction(0), MATRIX[I][J])

    F, P, R, QI = [MATRICES[NAME] for NAME in ('F', 'P', 'R', 'Q_i')]
    require(all(P[I][J] == P[J][I] for I in range(15) for J in range(15)), 'P must be symmetric')
    REFERENCE = reference_covariance(F, P, R, QI)
    for NAME, NODE in PROGRAM:
        VALUES[NAME] = evaluate_expression(NODE, VALUES, BITS)

    MAX_RATIO = 0.0
    MAX_ERROR = Fraction(0)
    MAX_BOUND = Fraction(0)
    MAX_DENSE_ERROR = Fraction(0)
    for I in range(15):
        for J in range(15):
            EXACT, BOUND, ROUNDED = VALUES['out', I, J]
            require(EXACT == REFERENCE[I][J], f'reference mismatch at {I},{J}')
            ACTUAL = decode_coefficient(RECORD['output'][15 * I + J], BITS)
            require(ACTUAL == ROUNDED, f'IEEE simulation mismatch at {I},{J}')
            ERROR = abs(ACTUAL - REFERENCE[I][J])
            require(ERROR <= BOUND, f'error bound exceeded at {I},{J}')
            require(
                RECORD['output'][15 * I + J] == RECORD['output'][15 * J + I],
                'mirror bit mismatch',
            )
            MAX_RATIO = max(MAX_RATIO, float(ERROR / BOUND) if BOUND else 0.0)
            MAX_ERROR = max(MAX_ERROR, ERROR)
            MAX_BOUND = max(MAX_BOUND, BOUND)
            if DENSE is not None:
                DENSE_VALUE = decode_coefficient(DENSE['output'][15 * I + J], BITS)
                MAX_DENSE_ERROR = max(MAX_DENSE_ERROR, abs(DENSE_VALUE - REFERENCE[I][J]))

    SCALE = max(abs(VALUE) for ROW in REFERENCE for VALUE in ROW)
    return {
        'max_error_to_bound': MAX_RATIO,
        'max_absolute_error': float(MAX_ERROR),
        'max_absolute_bound': float(MAX_BOUND),
        'max_matrix_scaled_error': float(MAX_ERROR / SCALE) if SCALE else 0.0,
        'dense_max_matrix_scaled_error': float(MAX_DENSE_ERROR / SCALE) if SCALE else 0.0,
    }
