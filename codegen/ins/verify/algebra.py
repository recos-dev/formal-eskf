"""Check emitted arithmetic against an independent exact-real block model."""

import sympy as S

from . import require
from .cpp import parse


def exact_model():
    """Return matrix input bindings and the dense covariance reference.

    Assemble the documented blocks independently of ins.covariance.model.
    """
    DT = S.Symbol('delta_t')
    F = S.zeros(15)
    P = S.Matrix(15, 15, lambda I, J: S.Symbol(f'prior_{min(I, J)}_{max(I, J)}'))
    R = S.Matrix(3, 3, lambda I, J: S.Symbol(f'rotation_{I}_{J}'))
    NOISE = S.zeros(15)
    QI = S.diag(*[S.Symbol(f'impulse_{I}') for I in range(12)])
    for I in range(15):
        if not 6 <= I < 9:
            F[I, I] = 1
    for I in range(3):
        F[I, I + 3] = DT
        F[I + 6, I + 12] = -DT
        for J in range(3):
            F[I + 3, J + 6] = S.Symbol(f'force_{I}_{J}')
            F[I + 3, J + 9] = S.Symbol(f'bias_{I}_{J}')
            F[I + 6, J + 6] = S.Symbol(f'attitude_{I}_{J}')
            NOISE[I + 3, J + 3] = sum(R[I, K] * QI[K, K] * R[J, K] for K in range(3))
    for I in range(6, 15):
        NOISE[I, I] = QI[I - 3, I - 3]
    INPUTS = {
        (NAME, I, J): MATRIX[I, J]
        for NAME, MATRIX in [('F', F), ('P', P), ('R', R), ('Q_i', QI)]
        for I in range(MATRIX.rows)
        for J in range(MATRIX.cols)
    }
    return INPUTS, F * P * F.T + NOISE


def symbolic_expression(NODE, VALUES):
    OPERATOR = NODE[0]
    if OPERATOR == 'ref':
        return VALUES[NODE[1]]
    if OPERATOR == 'int':
        return S.Integer(NODE[1])
    if OPERATOR == 'neg':
        return -symbolic_expression(NODE[1], VALUES)
    LEFT = symbolic_expression(NODE[1], VALUES)
    RIGHT = symbolic_expression(NODE[2], VALUES)
    if OPERATOR == 'add':
        return LEFT + RIGHT
    if OPERATOR == 'sub':
        return LEFT - RIGHT
    if OPERATOR == 'mul':
        return LEFT * RIGHT
    raise ValueError(f'unsupported operator: {OPERATOR}')


def verify_identities(TEXT):
    """Check every parsed output coefficient as a polynomial identity."""
    VALUES, REFERENCE = exact_model()
    PROGRAM = parse(TEXT)
    OUTPUTS = set()
    for NAME, NODE in PROGRAM:
        require(NAME not in VALUES, 'duplicate variable')
        if isinstance(NAME, tuple):
            OUTPUTS.add(NAME[1:])
        VALUES[NAME] = S.expand(symbolic_expression(NODE, VALUES))
    require(
        OUTPUTS == {(I, J) for I in range(15) for J in range(15)},
        'incomplete output',
    )
    for I, J in sorted(OUTPUTS):
        if S.expand(VALUES['out', I, J] - REFERENCE[I, J]) != 0:
            raise ValueError(f'polynomial mismatch at {I},{J}')
    return len(OUTPUTS)
