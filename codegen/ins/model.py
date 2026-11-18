"""INS15 equations before arithmetic scheduling or C++ emission.

Error blocks: [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
q_nb maps body to navigation; attitude error is right-local. Gravity is fixed.
The transition is the specified truncated model, not the derivative of the
complete constant-acceleration nominal integrator.

The schedulers lower these matrix equations into sparse, staged products:
    prediction: F P F^T + F_i Q_i F_i^T
    innovation: C = P H^T; S = H C + V
    Joseph: A P A^T + K_eff V K_eff^T, with A = I - K_eff H
    reset: G P G^T, with G = diag(I6, J, I6)
"""

import sympy as S

STATE_BLOCKS = ('position', 'velocity', 'attitude', 'accel_bias', 'gyro_bias')
NOISE_BLOCKS = ('specific_force', 'angular_rate', 'accel_bias', 'gyro_bias')
LAYOUTS = {
    'horizontal_position': (0, 1),
    'vertical_position': (2,),
    'horizontal_velocity': (3, 4),
    'velocity': (3, 4, 5),
    'body_vector': (6, 7, 8),
}


def matrix(NAME, ROWS, COLS):
    return S.Matrix(ROWS, COLS, lambda I, J: S.Symbol(f'{NAME}{I}_{J}'))


def symmetric(NAME, SIZE):
    return S.Matrix(SIZE, SIZE, lambda I, J: S.Symbol(f'{NAME}{min(I,J)}_{max(I,J)}'))


def hat(V):
    X, Y, Z = V
    return S.Matrix([[0, -Z, Y], [Z, 0, -X], [-Y, X, 0]])


def prediction_model():
    """Preserve the pinned scalar symbols while exposing the five-block F.

    force = -R hat(f_m-b_a) dt; bias = -R dt.
    attitude = R(Exp((omega_m-b_g)dt))^T, or its configured first-order form.
    These checked numerical producers remain in C++; they are model inputs here.
    """
    DT = S.Symbol('dt')
    I, Z = S.eye(3), S.zeros(3)
    FORCE = S.Matrix(3, 3, lambda I, J: S.Symbol(f'a{I}{J}'))
    BIAS = S.Matrix(3, 3, lambda I, J: S.Symbol(f'b{I}{J}'))
    ATTITUDE = S.Matrix(3, 3, lambda I, J: S.Symbol(f't{I}{J}'))
    F = S.BlockMatrix(
        [
            [I, DT * I, Z, Z, Z],
            [Z, I, FORCE, BIAS, Z],
            [Z, Z, ATTITUDE, Z, -DT * I],
            [Z, Z, Z, I, Z],
            [Z, Z, Z, Z, I],
        ]
    ).as_explicit()
    P = symmetric('p', 15)
    R = S.Matrix(3, 3, lambda I, J: S.Symbol(f'r{I}{J}'))
    Q = [S.Symbol(f'q{I}') for I in range(12)]
    return F, P, R, Q


def noise_injection(R):
    I, Z = S.eye(3), S.zeros(3)
    return S.BlockMatrix(
        [
            [Z, Z, Z, Z],
            [-R, Z, Z, Z],
            [Z, -I, Z, Z],
            [Z, Z, I, Z],
            [Z, Z, Z, I],
        ]
    ).as_explicit()


def observation(LAYOUT, H_BODY=None):
    """H = dh/d(delta_x), with accel bias preprocessing held fixed.

    Body-vector observations use h=-R^T g (accel) or h=R^T m_n (mag).
    Their residuals/policies stay in the caller; both have H_theta=hat(h).
    """
    COLS = LAYOUTS[LAYOUT]
    H = S.zeros(len(COLS), 15)
    if LAYOUT == 'body_vector':
        H[:, 6:9] = hat(H_BODY)
    else:
        for I, COL in enumerate(COLS):
            H[I, COL] = 1
    return H


def reset_transform(J):
    I, Z = S.eye(3), S.zeros(3)
    return S.BlockMatrix(
        [
            [I, Z, Z, Z, Z],
            [Z, I, Z, Z, Z],
            [Z, Z, J, Z, Z],
            [Z, Z, Z, I, Z],
            [Z, Z, Z, Z, I],
        ]
    ).as_explicit()
