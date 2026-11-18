#!/usr/bin/env python3
"""Check the physical local-error model before lowering it to scalar products."""

import json
from pathlib import Path
import sympy as S

from ins.model import hat, observation, prediction_model, noise_injection, reset_transform, LAYOUTS
from ins.verify import require


def zero(MATRIX):
    require(all(S.expand(X) == 0 for X in MATRIX), 'symbolic model mismatch')


def main():
    Q = S.Matrix(S.symbols('q0 q1 q2 q3', real=True))
    Q0 = Q[0]
    V = Q[1:4, 0]
    R = (Q0**2 - (V.T * V)[0]) * S.eye(3) + 2 * V * V.T + 2 * Q0 * hat(V)
    # Unit q, scalar first, q_true=q*Exp(delta_theta). This is the derivative
    # at delta_theta=0, not a finite-angle replacement of Exp.
    DQ = S.Rational(1, 2) * S.Matrix.vstack(-V.T, Q0 * S.eye(3) + hat(V))
    REFERENCE = S.Matrix(S.symbols('r_N r_E r_D', real=True))
    H_PREDICTED = R.T * REFERENCE
    DERIVATIVE = H_PREDICTED.jacobian(Q) * DQ
    zero(DERIVATIVE - hat(H_PREDICTED))
    # r_n can be m_n or -g_n. The gravity model keeps bias preprocessing fixed.
    zero(observation('body_vector', H_PREDICTED)[:, 6:9] - DERIVATIVE)

    DELTA = S.Matrix(
        S.symbols(
            'dp_N dp_E dp_D dv_N dv_E dv_D dtheta_x dtheta_y dtheta_z dba_x dba_y dba_z dbg_x dbg_y dbg_z'
        )
    )
    FORCE = S.Matrix(S.symbols('f_x f_y f_z'))
    RATE = S.Matrix(S.symbols('omega_x omega_y omega_z'))
    DYNAMICS = S.Matrix.vstack(
        DELTA[3:6, 0],
        -R * hat(FORCE) * DELTA[6:9, 0] - R * DELTA[9:12, 0],
        -hat(RATE) * DELTA[6:9, 0] - DELTA[12:15, 0],
        S.zeros(6, 1),
    )
    CONTINUOUS = DYNAMICS.jacobian(DELTA)
    F, _, _, _ = prediction_model()
    DT = S.Symbol('dt')
    REPLACEMENTS = {}
    for I in range(3):
        for J in range(3):
            REPLACEMENTS[S.Symbol(f'a{I}{J}')] = (-R * hat(FORCE) * DT)[I, J]
            REPLACEMENTS[S.Symbol(f'b{I}{J}')] = (-R * DT)[I, J]
            REPLACEMENTS[S.Symbol(f't{I}{J}')] = (S.eye(3) - hat(RATE) * DT)[I, J]
    zero(F.subs(REPLACEMENTS, simultaneous=True) - (S.eye(15) + CONTINUOUS * DT))
    # Exact attitude mode replaces only the 3x3 attitude block; bias/position
    # coupling stays truncated. It is not an exact joint discretization.
    QI = S.diag(*S.symbols('Qa_x Qa_y Qa_z Qw_x Qw_y Qw_z Qba_x Qba_y Qba_z Qbg_x Qbg_y Qbg_z'))
    FI = noise_injection(R)
    EXPECTED = S.zeros(15)
    EXPECTED[3:6, 3:6] = R * QI[:3, :3] * R.T
    EXPECTED[6:15, 6:15] = QI[3:12, 3:12]
    zero(FI * QI * FI.T - EXPECTED)
    J = S.Matrix(3, 3, lambda I, K: S.Symbol(f'J{I}{K}'))
    G = reset_transform(J)
    require(G[6:9, 6:9] == J and G[:6, :6] == S.eye(6) and G[9:, 9:] == S.eye(6), 'reset ordering')
    for LAYOUT, COLS in LAYOUTS.items():
        H = observation(LAYOUT, H_PREDICTED if LAYOUT == 'body_vector' else None)
        K = S.Matrix(15, len(COLS), lambda I, J: S.Symbol(f'k{I}_{J}'))
        A = S.eye(15) - K * H
        require(
            all(
                A[I, J] == S.Integer(I == J) for I in range(15) for J in range(15) if J not in COLS
            ),
            'Joseph inactive column premise',
        )
    REPORT = {
        'checks': [
            'scalar-first right-local observation derivative',
            'fixed accel-bias preprocessing Jacobian',
            'truncated first-order error transition',
            'anisotropic noise injection',
            'reset block ordering',
            'Joseph inactive-column structure for every observation layout',
        ],
        'scope': 'SymPy polynomial checks. Existing SO(3) proofs own Exp and exact right-Jacobian derivatives.',
    }
    DEST = Path(__file__).resolve().parents[3] / 'output/codegen/products/derivation.json'
    DEST.parent.mkdir(parents=True, exist_ok=True)
    DEST.write_text(json.dumps(REPORT, indent=2) + '\n')
    print('Physical model: 6 checks passed')


if __name__ == '__main__':
    main()
