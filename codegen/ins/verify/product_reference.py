"""Independent input contracts and dense references, not generator models."""

import sympy as S

LAYOUTS = {
    'horizontal_position': (0, 1),
    'vertical_position': (2,),
    'horizontal_velocity': (3, 4),
    'velocity': (3, 4, 5),
    'body_vector': (6, 7, 8),
}


def contracts():
    RESULT = {
        'ins_reset': dict(
            group='reset',
            dimensions={'P': (15, 15), 'J': (3, 3)},
            shape=(15, 15),
            symmetric=('P',),
            mirror=True,
            layout=None,
            kind='reset',
        )
    }
    for LAYOUT, COLS in LAYOUTS.items():
        M = len(COLS)
        EXTRA = {'h': (3, 1)} if LAYOUT == 'body_vector' else {}
        for KIND, DIMS, SHAPE, SYMMETRIC, MIRROR in (
            ('cross', {'P': (15, 15), **EXTRA}, (15, M), ('P',), False),
            ('innovation', {'C': (15, M), **EXTRA, 'V': (M, M)}, (M, M), ('V',), False),
            (
                'joseph',
                {'P': (15, 15), 'A': (15, 15), 'K': (15, M), 'V': (M, M)},
                (15, 15),
                ('P', 'V'),
                True,
            ),
        ):
            RESULT[f'ins_{LAYOUT}_{KIND}'] = dict(
                group='joseph' if KIND == 'joseph' else 'innovation',
                dimensions=DIMS,
                shape=SHAPE,
                symmetric=SYMMETRIC,
                mirror=MIRROR,
                layout=LAYOUT,
                kind=KIND,
            )
    return RESULT


def model(SPEC):
    MATRICES, BINDINGS = {}, {}
    for NAME, (ROWS, COLS) in SPEC['dimensions'].items():

        def coefficient(I, J):
            if NAME in SPEC['symmetric']:
                I, J = min(I, J), max(I, J)
            if NAME == 'A' and J not in LAYOUTS[SPEC['layout']]:
                return S.Integer(I == J)
            return S.Symbol(f'{NAME}_{I}_{J}')

        MATRICES[NAME] = S.Matrix(ROWS, COLS, coefficient)
        BINDINGS.update(
            {(NAME, I, J): MATRICES[NAME][I, J] for I in range(ROWS) for J in range(COLS)}
        )
    if SPEC['kind'] == 'reset':
        G = S.eye(15)
        G[6:9, 6:9] = MATRICES['J']
        REFERENCE = G * MATRICES['P'] * G.T
    else:
        COLS = LAYOUTS[SPEC['layout']]
        H = S.zeros(len(COLS), 15)
        if SPEC['layout'] == 'body_vector' and SPEC['kind'] != 'joseph':
            X, Y, Z = MATRICES['h']
            H[:, 6:9] = S.Matrix([[0, -Z, Y], [Z, 0, -X], [-Y, X, 0]])
        else:
            for I, COL in enumerate(COLS):
                H[I, COL] = 1
        if SPEC['kind'] == 'cross':
            REFERENCE = MATRICES['P'] * H.T
        elif SPEC['kind'] == 'innovation':
            REFERENCE = H * MATRICES['C'] + MATRICES['V']
        else:
            A = MATRICES['A']
            REFERENCE = A * MATRICES['P'] * A.T + MATRICES['K'] * MATRICES['V'] * MATRICES['K'].T
    return BINDINGS, REFERENCE
