/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "correction_support.hpp"
#include <formal_eskf/so3/operations.hpp>
#define private public
#include <formal_eskf/runtime/tilt_alignment.hpp>
#undef private
namespace tilt_proof
{
using correction_proof::finite_matrix;
using correction_proof::same;
using correction_proof::same_matrix;
using correction_proof::Scalar;
using formal_eskf::Status;
using solve_proof::Backend;
using Matrix = formal_eskf::linalg::Matrix<Backend, 3U, 3U>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
using Alignment = formal_eskf::runtime::TiltAlignment<Backend>;
struct Calls
{
    inline static Quaternion q;
    inline static Matrix P, rotation, middle, world;
    inline static unsigned rotations{}, products{};
};
} // namespace tilt_proof
// Reuse the exact-type E-OBS rotation and E-CORRECT 3x3 product producers.
// Arbitrary IEEE output matrices overapproximate their numeric results;
// exact operand/order/purity checks compose their proved value frames.
namespace formal_eskf::so3
{
template <> inline tilt_proof::Matrix to_rotation_matrix<tilt_proof::Backend>(tilt_proof::Quaternion const & q)
{
    using namespace tilt_proof;
    __ESBMC_assert(
        Calls::rotations++ == 0U && Calls::products == 0U && same_matrix(q.coefficients(), Calls::q.coefficients()),
        "R-ACCELEROMETER-CONDITIONS: alignment rotates the original quaternion exactly once before products");
    return Calls::rotation;
}
} // namespace formal_eskf::so3
namespace formal_eskf::linalg
{
template <>
template <>
inline tilt_proof::Matrix Matrix<tilt_proof::Backend, 3U, 3U>::operator*<3U>(tilt_proof::Matrix const & other) const
{
    using namespace tilt_proof;
    __ESBMC_assert(Calls::rotations == 1U && Calls::products < 2U,
                   "R-ACCELEROMETER-CONDITIONS: exactly two products follow quaternion conversion");
    if (Calls::products++ == 0U)
    {
        __ESBMC_assert(same_matrix(*this, Calls::rotation) && same_matrix(other, Calls::P),
                       "R-ACCELEROMETER-CONDITIONS: first covariance product is R_nb times P_theta_b");
        return Calls::middle;
    }
    __ESBMC_assert(same_matrix(*this, Calls::middle),
                   "R-ACCELEROMETER-CONDITIONS: second product consumes the first product unchanged");
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(other(row, column), Calls::rotation(column, row)),
                           "R-ACCELEROMETER-CONDITIONS: right covariance factor is the complete transpose of R_nb");
        }
    }
    return Calls::world;
}
} // namespace formal_eskf::linalg
void verify_tilt_initialization()
{
    using namespace tilt_proof;
    Alignment alignment;
    __ESBMC_assert(!alignment.aligned() && same(alignment.variance(), Scalar{0}),
                   "R-ACCELEROMETER-CONDITIONS: alignment starts unlatched with zero variance");
}
void verify_tilt_alignment(tilt_proof::Alignment alignment, tilt_proof::Quaternion q, tilt_proof::Matrix P,
                           tilt_proof::Matrix rotation, tilt_proof::Matrix middle, tilt_proof::Matrix world)
{
    using namespace tilt_proof;
    auto const before = alignment;
    auto const old_q = q;
    auto const old_P = P;
    Calls::q = q;
    Calls::P = P;
    Calls::rotation = rotation;
    Calls::middle = middle;
    Calls::world = world;
    Calls::rotations = 0U;
    Calls::products = 0U;
    bool const valid = finite_matrix(q.coefficients()) && finite_matrix(P);
    Scalar const variance = world(0U, 0U) + world(1U, 1U);
    Status const expected = !valid                                                   ? Status::non_finite_input
                            : !finite_matrix(world) || !std::isfinite(variance)      ? Status::non_finite_result
                            : world(0U, 0U) < Scalar{0} || world(1U, 1U) < Scalar{0} ? Status::domain_error
                                                                                     : Status::success;
    constexpr Scalar limit = std::numbers::pi_v<Scalar> / Scalar{60};
    auto const result = alignment.update(q, P);
    __ESBMC_assert(result == expected && Calls::rotations == (valid ? 1U : 0U) && Calls::products == (valid ? 2U : 0U),
                   "R-ACCELEROMETER-CONDITIONS: alignment validates inputs before arithmetic and preserves exact "
                   "failure priority");
    if (result == Status::success)
    {
        __ESBMC_assert(same(alignment.variance(), variance) &&
                           alignment.aligned() == (before.m_aligned || variance < limit * limit) &&
                           std::isfinite(alignment.variance()) && alignment.variance() >= Scalar{0},
                       "R-ACCELEROMETER-CONDITIONS: only N/E variance drives the strict three-degree alignment latch");
    }
    else
    {
        __ESBMC_assert(same(alignment.variance(), before.m_variance) && alignment.aligned() == before.m_aligned,
                       "R-ACCELEROMETER-CONDITIONS: every alignment failure preserves both history fields");
    }
    __ESBMC_assert(
        !before.m_aligned || alignment.aligned(),
        "R-ACCELEROMETER-CONDITIONS: established tilt alignment cannot be cleared by variance growth or failure");
    __ESBMC_assert(!(std::isfinite(before.m_variance) && before.m_variance >= Scalar{0}) ||
                       (std::isfinite(alignment.variance()) && alignment.variance() >= Scalar{0}),
                   "R-ACCELEROMETER-CONDITIONS: finite nonnegative variance is an initialized inductive invariant");
    __ESBMC_assert(same_matrix(q.coefficients(), old_q.coefficients()) && same_matrix(P, old_P),
                   "R-ACCELEROMETER-CONDITIONS: alignment preserves the borrowed attitude and covariance");
}
int main() { return 0; }
