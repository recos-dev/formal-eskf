"""Generate the INS covariance upper triangle with deterministic staged CSE.

The input producers retain the core's right-local INS15 formulation. Only the
scalar evaluation of F P Fᵀ + G Q_i Gᵀ is generated here.
"""

import sympy as S


from .model import prediction_model as model


def input_bindings():
    """Bind symbolic inputs to fixed C++ matrix entries in emission order."""
    INPUTS = {'dt': 'F(0U, 3U)'}
    for I in range(3):
        for J in range(3):
            for PREFIX, ROW, COL in [('a', 3, 6), ('b', 3, 9), ('t', 6, 6)]:
                INPUTS[f'{PREFIX}{I}{J}'] = f'F({ROW + I}U, {COL + J}U)'
            INPUTS[f'r{I}{J}'] = f'R({I}U, {J}U)'
    for I in range(15):
        for J in range(I, 15):
            INPUTS[f'p{I}_{J}'] = f'P({I}U, {J}U)'
    for I in range(12):
        INPUTS[f'q{I}'] = f'Q_i({I}U, {I}U)'
    return INPUTS


def scalar_stages():
    """Return ordered temporaries and upper outputs for the two CSE stages."""
    F, P, R, Q = model()
    # Only scalar components consumed by the upper triangle are materialized.
    NEEDED = sorted(
        {(I, K) for I in range(15) for J in range(I, 15) for K in range(15) if F[J, K] != 0}
    )
    W = {(I, K): S.Symbol(f'w{I}_{K}') for I, K in NEEDED}
    FIRST_NAMES = list(W.values())
    FIRST_VALUES = [sum(F[I, L] * P[L, K] for L in range(15)) for I, K in NEEDED]
    NOISE = {}
    for I in range(3):
        for K in range(3):
            NOISE[I, K] = S.Symbol(f'n{I}_{K}')
            FIRST_NAMES.append(NOISE[I, K])
            FIRST_VALUES.append(R[I, K] * Q[K])

    UPPER = [(I, J) for I in range(15) for J in range(I, 15)]
    SECOND_VALUES = []
    for I, J in UPPER:
        VALUE = sum(W[I, K] * F[J, K] for K in range(15) if F[J, K] != 0)
        if 3 <= I <= J < 6:
            VALUE += sum(NOISE[I - 3, K] * R[J - 3, K] for K in range(3))
        elif I == J and I >= 6:
            VALUE += Q[I - 3]
        SECOND_VALUES.append(VALUE)

    FIRST_COMMON, FIRST_VALUES = S.cse(
        FIRST_VALUES, symbols=S.numbered_symbols('c'), order='canonical'
    )
    SECOND_COMMON, SECOND_VALUES = S.cse(
        SECOND_VALUES, symbols=S.numbered_symbols('d'), order='canonical'
    )
    TEMPORARIES = FIRST_COMMON + list(zip(FIRST_NAMES, FIRST_VALUES)) + SECOND_COMMON
    return TEMPORARIES, list(zip(UPPER, SECOND_VALUES))
