## Summary

Define the state layout, frames, units, and perturbation convention used by the ESKF.

These conventions are shared by prediction, correction, measurement models, state injection, and covariance reset.

## Supported configurations

| Configuration | Nominal state | Nominal coefficients | Error state | Error dimension / P |
|---|---|---|---|---|
| INS | [p_n, v_n, q_nb, b_a, b_g] | 16 | [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g] | 15 / 15x15 |
| AHRS | [q_nb] | 4 | [delta_theta_b] | 3 / 3x3 |

These are logical coefficient layouts, not a C++ memory-layout or serialization contract.

## AHRS state

AHRS contains rotation only. Its nominal coefficients are `q_nb = [q0, q1, q2, q3]`, and its error coordinates are `delta_theta_b`. It does not estimate position, velocity or IMU biases; gyroscope bias calibration is external to this configuration.

~~~text
q_true = q_nb ⊗ Exp_q(delta_theta_b)
q_nb  <- q_nb ⊗ Exp_q(estimated_delta_theta_b)
P = Cov(delta_theta_b) in R^(3x3)
~~~

The attitude error is local and right-multiplicative, as in INS. The covariance describes the three-dimensional tangent error, not four quaternion coefficients.

## INS nominal state

The nominal state has 16 scalar coefficients:

~~~text
x = [p_n, v_n, q_nb, b_a, b_g]

p_n  in R³
v_n  in R³
q_nb in S³
b_a  in R³
b_g  in R³
~~~

The coefficient layout is:

~~~text
0..2    p_n
3..5    v_n
6..9    q_nb = [q0, q1, q2, q3]
10..12  b_a
13..15  b_g
~~~

The quantities have the following meanings:

| Symbol | Meaning | Frame | Unit |
|---|---|---|---|
| `p_n` | Position relative to the local navigation origin | NED | m |
| `v_n` | Linear velocity | NED | m/s |
| `q_nb` | Rotation from body to navigation | body -> NED | unit quaternion |
| `b_a` | Accelerometer bias | body | m/s² |
| `b_g` | Gyroscope bias | body | rad/s |

## INS error state

The error state has 15 scalar coefficients:

~~~text
delta_x = [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g]
~~~

The coefficient layout is:

~~~text
0..2    delta_p_n
3..5    delta_v_n
6..8    delta_theta_b
9..11   delta_b_a
12..14  delta_b_g
~~~

`delta_theta_b` is a local rotation vector expressed in the body tangent frame. It is not a quaternion and does not add directly to the quaternion coefficients.

The INS covariance follows this error-state ordering:

~~~text
P = Cov(delta_x) in R^(15x15)
  = E[delta_x * transpose(delta_x)] when the error mean is zero
~~~

## INS true-state relationship

The true state is related to the nominal state by:

~~~text
p_true   = p_n + delta_p_n
v_true   = v_n + delta_v_n
q_true   = q_nb ⊗ Exp_q(delta_theta_b)
b_a_true = b_a + delta_b_a
b_g_true = b_g + delta_b_g
~~~

The attitude error is therefore local and right-multiplicative.

The zero error is:

~~~text
delta_x = 0
Exp_q(delta_theta_b) = identity
true state = nominal state
~~~

State injection uses the same relationship:

~~~text
p_n  <- p_n + estimated_delta_p_n
v_n  <- v_n + estimated_delta_v_n
q_nb <- q_nb ⊗ Exp_q(estimated_delta_theta_b)
b_a  <- b_a + estimated_delta_b_a
b_g  <- b_g + estimated_delta_b_g
~~~

The covariance-reset transformation after injection is specified separately.

## Frame and sign conventions

The navigation frame is NED:

~~~text
x = North
y = East
z = Down
~~~

Gravity therefore has positive navigation-frame `z`:

~~~text
g_n = [0, 0, +g]
~~~

`q_nb` and `R_nb` map body coordinates into navigation coordinates:

~~~text
v_n = R_nb * v_b
v_n = rotate(q_nb, v_b)
~~~

Accelerometer and gyroscope measurements, including their biases, are expressed in body coordinates.

All angular quantities used inside the ESKF are in radians. Degrees are allowed only at external input/output boundaries with explicit conversion.

## State validity

A valid nominal state satisfies:

~~~text
all state coefficients are finite
q_nb is a unit quaternion
all fields present in the selected configuration use the defined frames and units
all components describe the same logical timestamp
~~~

A valid covariance satisfies:

~~~text
P has dimension 15x15 for INS or 3x3 for AHRS
P uses the defined error-state ordering
P is symmetric and positive semidefinite in the mathematical model
all coefficients are finite
~~~

Quaternion sign equivalence applies:

~~~text
q_nb and -q_nb represent the same nominal attitude
~~~

Changing the state order, frame direction, quaternion convention, or perturbation side is an algorithm-wide semantic change.
