"""Schedule the fixed INS products; equations live separately in model.py."""

import sympy as S

from ..model import LAYOUTS, matrix, symmetric, observation


def inputs(DIMENSIONS, SYMMETRIC=()):
    MATRICES, BINDINGS = {}, {}
    for NAME, (ROWS, COLS) in DIMENSIONS.items():
        PREFIX = NAME.lower()
        VALUE = symmetric(PREFIX, ROWS) if NAME in SYMMETRIC else matrix(PREFIX, ROWS, COLS)
        MATRICES[NAME] = VALUE
        for I in range(ROWS):
            for J in range(COLS):
                if NAME not in SYMMETRIC or I <= J:
                    BINDINGS[VALUE[I, J]] = (NAME, I, J)
    return MATRICES, BINDINGS


def stage(EXPRESSIONS, NAMES, NAME):
    COMMON, VALUES = S.cse(EXPRESSIONS, symbols=S.numbered_symbols(f'c{NAME}_'), order='canonical')
    return COMMON + list(zip(NAMES, VALUES))


def product(NAME, GROUP, DIMENSIONS, SHAPE, BINDINGS, TEMPORARIES, VALUES, MIRROR=False):
    COMMON, OUTPUTS = S.cse(
        [V for _, V in VALUES], symbols=S.numbered_symbols('d'), order='canonical'
    )
    return dict(
        name=NAME,
        group=GROUP,
        dimensions=DIMENSIONS,
        shape=SHAPE,
        bindings=BINDINGS,
        temporaries=TEMPORARIES + COMMON,
        outputs=list(zip([IJ for IJ, _ in VALUES], OUTPUTS)),
        mirror=MIRROR,
    )


def reset():
    DIMENSIONS = {'P': (15, 15), 'J': (3, 3)}
    INPUTS, BINDINGS = inputs(DIMENSIONS, ('P',))
    P, J = INPUTS['P'], INPUTS['J']
    W = matrix('w', 3, 3)
    TEMPORARIES = stage(list(J * P[6:9, 6:9]), list(W), 0)
    VALUES = []
    for I in range(15):
        for L in range(I, 15):
            if 6 <= I <= L < 9:
                VALUE = sum(W[I - 6, K] * J[L - 6, K] for K in range(3))
            elif 6 <= I < 9:
                VALUE = sum(J[I - 6, K] * P[6 + K, L] for K in range(3))
            elif 6 <= L < 9:
                VALUE = sum(P[I, 6 + K] * J[L - 6, K] for K in range(3))
            else:
                VALUE = P[I, L]
            VALUES.append(((I, L), VALUE))
    return product('ins_reset', 'reset', DIMENSIONS, (15, 15), BINDINGS, TEMPORARIES, VALUES, True)


def correction(LAYOUT):
    M = len(LAYOUTS[LAYOUT])
    EXTRA = {'h': (3, 1)} if LAYOUT == 'body_vector' else {}
    DIMENSIONS = {'P': (15, 15), **EXTRA}
    INPUTS, BINDINGS = inputs(DIMENSIONS, ('P',))
    H = observation(LAYOUT, INPUTS.get('h'))
    C = INPUTS['P'] * H.T
    CROSS = product(
        f'ins_{LAYOUT}_cross',
        'innovation',
        DIMENSIONS,
        C.shape,
        BINDINGS,
        [],
        [((I, J), C[I, J]) for I in range(15) for J in range(M)],
    )

    DIMENSIONS = {'C': (15, M), **EXTRA, 'V': (M, M)}
    INPUTS, BINDINGS = inputs(DIMENSIONS, ('V',))
    H = observation(LAYOUT, INPUTS.get('h'))
    # Preserve the existing product-then-noise boundary. Flattening the sum
    # allows CSE to absorb small V into a term before cancellation in H*C.
    # Keep the same rule for every layout; no per-input ordering decisions.
    HC = matrix('hc', M, M)
    TEMPORARIES = stage(list(H * INPUTS['C']), list(HC), 'innovation')
    INNOVATION = HC + INPUTS['V']
    INNOVATION = product(
        f'ins_{LAYOUT}_innovation',
        'innovation',
        DIMENSIONS,
        (M, M),
        BINDINGS,
        TEMPORARIES,
        [((I, J), INNOVATION[I, J]) for I in range(M) for J in range(M)],
    )

    # The caller already computes/checks A = I - K_eff H before Joseph.
    # Consume that same intermediate rather than duplicating it or moving its
    # overflow check into unchecked generated arithmetic.
    DIMENSIONS = {'P': (15, 15), 'A': (15, 15), 'K': (15, M), 'V': (M, M)}
    INPUTS, BINDINGS = inputs(DIMENSIONS, ('P', 'V'))
    P, K, V = INPUTS['P'], INPUTS['K'], INPUTS['V']
    A = S.eye(15)
    COLUMNS = LAYOUTS[LAYOUT]
    for I in range(15):
        for J in COLUMNS:
            A[I, J] = INPUTS['A'][I, J]
    TEMPORARIES = []
    # Keep the two Joseph sandwiches staged. Never substitute an optimal-gain
    # identity or expand across the named A / W / noise intermediates.
    NEEDED = sorted(
        {(I, K) for I in range(15) for J in range(I, 15) for K in range(15) if A[J, K] != 0}
    )
    W = {IJ: S.Symbol(f'w{IJ[0]}_{IJ[1]}') for IJ in NEEDED}
    FIRST = [sum(A[I, L] * P[L, J] for L in range(15)) for I, J in NEEDED]
    N = matrix('n', 15, M)
    TEMPORARIES += stage(FIRST + list(K * V), list(W.values()) + list(N), 1)
    VALUES = [
        (
            (I, J),
            sum(W[I, L] * A[J, L] for L in range(15) if A[J, L] != 0)
            + sum(N[I, L] * K[J, L] for L in range(M)),
        )
        for I in range(15)
        for J in range(I, 15)
    ]
    JOSEPH = product(
        f'ins_{LAYOUT}_joseph', 'joseph', DIMENSIONS, (15, 15), BINDINGS, TEMPORARIES, VALUES, True
    )
    return CROSS, INNOVATION, JOSEPH


def kernels():
    return [reset()] + [KERNEL for LAYOUT in LAYOUTS for KERNEL in correction(LAYOUT)]
