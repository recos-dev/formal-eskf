/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/**
 * @file
 * Scalar-first unit-quaternion value type.
 */

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::so3
{

/**
 * Unit quaternion using scalar-first [q0, q1, q2, q3] coefficients and Hamilton
 * multiplication.
 *
 * The ordering corresponds to [qw, qx, qy, qz], and multiplication and
 * unit-quaternion inversion follow equations (12) and (30) of Joan Sola,
 * "Quaternion kinematics for the error-state Kalman filter".
 *
 * @see https://arxiv.org/abs/1711.02508
 *
 * Construction from arbitrary coefficients is checked and normalized.
 * Arithmetic evaluates eagerly.  Multiplication preserves unit norm over the
 * mathematical reals; finite-precision residual bounds belong to the selected
 * numerical profile.
 */
template <typename Linalg> class UnitQuaternion
{
public:
    using linalg_type = Linalg;
    using scalar_math_type = typename Linalg::scalar_math_type;
    using value_type = typename Linalg::value_type;
    using vector4_type = typename Linalg::template vector_type<4U>;

    constexpr UnitQuaternion() noexcept = default;
    UnitQuaternion(UnitQuaternion const &) = default;
    UnitQuaternion(UnitQuaternion &&) noexcept = default;
    UnitQuaternion & operator=(UnitQuaternion const &) = default;
    UnitQuaternion & operator=(UnitQuaternion &&) noexcept = default;
    ~UnitQuaternion() = default;

    [[nodiscard]] static constexpr UnitQuaternion identity() noexcept { return UnitQuaternion(); }

    [[nodiscard]] static Status try_from_coefficients(value_type q0, value_type q1, value_type q2, value_type q3,
                                                      value_type minimum_norm, UnitQuaternion & output) noexcept;

    [[nodiscard]] static Status try_from_coefficients(vector4_type const & coefficients, value_type minimum_norm,
                                                      UnitQuaternion & output) noexcept;

    [[nodiscard]] constexpr value_type q0() const noexcept { return m_q0; }
    [[nodiscard]] constexpr value_type q1() const noexcept { return m_q1; }
    [[nodiscard]] constexpr value_type q2() const noexcept { return m_q2; }
    [[nodiscard]] constexpr value_type q3() const noexcept { return m_q3; }

    [[nodiscard]] vector4_type coefficients() const noexcept;

    [[nodiscard]] constexpr UnitQuaternion operator-() const noexcept;
    [[nodiscard]] constexpr UnitQuaternion operator*(UnitQuaternion const & other) const noexcept;
    [[nodiscard]] constexpr UnitQuaternion inverse() const noexcept;

private:
    struct Unchecked
    {
    };

    constexpr UnitQuaternion(value_type q0, value_type q1, value_type q2, value_type q3, Unchecked) noexcept
        : m_q0(q0), m_q1(q1), m_q2(q2), m_q3(q3)
    {
    }

    value_type m_q0{1};
    value_type m_q1{0};
    value_type m_q2{0};
    value_type m_q3{0};
};

template <typename Linalg>
Status UnitQuaternion<Linalg>::try_from_coefficients(value_type q0, value_type q1, value_type q2, value_type q3,
                                                     value_type minimum_norm, UnitQuaternion & output) noexcept
{
    vector4_type coefficients;
    coefficients.set(0U, q0);
    coefficients.set(1U, q1);
    coefficients.set(2U, q2);
    coefficients.set(3U, q3);
    return try_from_coefficients(coefficients, minimum_norm, output);
}

template <typename Linalg>
Status UnitQuaternion<Linalg>::try_from_coefficients(vector4_type const & coefficients, value_type minimum_norm,
                                                     UnitQuaternion & output) noexcept
{
    if (!scalar::is_finite<scalar_math_type>(minimum_norm))
    {
        return Status::non_finite_input;
    }
    if (!(minimum_norm > value_type{0}))
    {
        return Status::domain_error;
    }

    vector4_type normalized;
    Status const status = linalg::try_normalize(coefficients, minimum_norm, normalized);
    if (status == Status::zero_or_unsafe_divisor)
    {
        return Status::invalid_quaternion_norm;
    }
    if (!succeeded(status))
    {
        return status;
    }

    vector4_type const & result = normalized;
    output.m_q0 = result(0U);
    output.m_q1 = result(1U);
    output.m_q2 = result(2U);
    output.m_q3 = result(3U);
    return Status::success;
}

template <typename Linalg>
typename UnitQuaternion<Linalg>::vector4_type UnitQuaternion<Linalg>::coefficients() const noexcept
{
    vector4_type result;
    result.set(0U, m_q0);
    result.set(1U, m_q1);
    result.set(2U, m_q2);
    result.set(3U, m_q3);
    return result;
}

template <typename Linalg> constexpr UnitQuaternion<Linalg> UnitQuaternion<Linalg>::operator-() const noexcept
{
    return UnitQuaternion(-m_q0, -m_q1, -m_q2, -m_q3, Unchecked{});
}

template <typename Linalg>
constexpr UnitQuaternion<Linalg> UnitQuaternion<Linalg>::operator*(UnitQuaternion const & other) const noexcept
{
    return UnitQuaternion(m_q0 * other.m_q0 - m_q1 * other.m_q1 - m_q2 * other.m_q2 - m_q3 * other.m_q3,
                          m_q0 * other.m_q1 + m_q1 * other.m_q0 + m_q2 * other.m_q3 - m_q3 * other.m_q2,
                          m_q0 * other.m_q2 - m_q1 * other.m_q3 + m_q2 * other.m_q0 + m_q3 * other.m_q1,
                          m_q0 * other.m_q3 + m_q1 * other.m_q2 - m_q2 * other.m_q1 + m_q3 * other.m_q0, Unchecked{});
}

template <typename Linalg> constexpr UnitQuaternion<Linalg> UnitQuaternion<Linalg>::inverse() const noexcept
{
    return UnitQuaternion(m_q0, -m_q1, -m_q2, -m_q3, Unchecked{});
}

} /* end namespace formal_eskf::so3 */
