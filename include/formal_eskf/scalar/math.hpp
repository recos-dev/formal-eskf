#pragma once

/**
 * @file
 * Backend-independent checked scalar-math operations.
 */

#include <formal_eskf/status.hpp>

namespace formal_eskf::scalar
{

template <typename Scalar> struct SinCos
{
    Scalar sine{};
    Scalar cosine{};
};

template <typename Math> [[nodiscard]] constexpr typename Math::value_type pi() noexcept { return Math::pi(); }

template <typename Math> [[nodiscard]] bool is_finite(typename Math::value_type value) noexcept
{
    return Math::is_finite(value);
}

template <typename Math> [[nodiscard]] typename Math::value_type absolute(typename Math::value_type value) noexcept
{
    return Math::absolute(value);
}

/** Compute a square root without changing output on failure. */
template <typename Math>
[[nodiscard]] Status try_sqrt(typename Math::value_type input, typename Math::value_type & output) noexcept
{
    using value_type = typename Math::value_type;

    if (!is_finite<Math>(input))
    {
        return Status::non_finite_input;
    }
    if (input < value_type{0})
    {
        return Status::domain_error;
    }

    value_type const candidate = Math::sqrt(input);
    if (!is_finite<Math>(candidate))
    {
        return Status::non_finite_result;
    }

    output = candidate;
    return Status::success;
}

/** Compute sine and cosine together without changing output on failure. */
template <typename Math>
[[nodiscard]] Status try_sin_cos(typename Math::value_type angle, SinCos<typename Math::value_type> & output) noexcept
{
    using value_type = typename Math::value_type;

    if (!is_finite<Math>(angle))
    {
        return Status::non_finite_input;
    }

    SinCos<value_type> const candidate{Math::sin(angle), Math::cos(angle)};
    if (!is_finite<Math>(candidate.sine) || !is_finite<Math>(candidate.cosine))
    {
        return Status::non_finite_result;
    }

    output = candidate;
    return Status::success;
}

/** Compute atan2(y, x) without changing output on failure. */
template <typename Math>
[[nodiscard]] Status try_atan2(typename Math::value_type y, typename Math::value_type x,
                               typename Math::value_type & output) noexcept
{
    using value_type = typename Math::value_type;

    if (!is_finite<Math>(y) || !is_finite<Math>(x))
    {
        return Status::non_finite_input;
    }
    if (y == value_type{0} && x == value_type{0})
    {
        return Status::domain_error;
    }

    value_type const candidate = Math::atan2(y, x);
    if (!is_finite<Math>(candidate))
    {
        return Status::non_finite_result;
    }

    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::scalar */
