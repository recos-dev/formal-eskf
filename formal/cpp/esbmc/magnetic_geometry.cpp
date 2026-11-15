/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "step_correction_support.hpp"
#include <formal_eskf/eskf/magnetometer.hpp>

static_assert(ESKF_MAG_TILT == 1 && step_correction_proof::boundaries_clear &&
              correction_proof::measurement_size == 3U);
using namespace step_correction_proof;
using correction_proof::finite_matrix;
using correction_proof::Gain;
constexpr std::size_t attitude_offset = state_size == 15U ? 6U : 0U;

// Actual products, classification and storage. The only summary in this file
// is the rotation producer used by prepare; its same-type E-OBS producer is
// required evidence. Projection entry points do not call that summary.
struct RotationCall
{
    inline static Matrix3 result;
    inline static Quaternion argument;
    inline static unsigned count{};
};
namespace formal_eskf::so3
{
template <> inline Matrix3 to_rotation_matrix<Backend>(Quaternion const & q)
{
    __ESBMC_assert(RotationCall::count++ == 0U && same_matrix(q.coefficients(), RotationCall::argument.coefficients()),
                   "E-MAG-TILT: observation uses the original quaternion exactly once");
    return RotationCall::result;
}
} // namespace formal_eskf::so3

void verify_magnetic_observation(Quaternion q, Vector3 measurement, Vector3 reference, Matrix3 rotation)
{
    auto const old_q = q;
    auto const old_measurement = measurement;
    auto const old_reference = reference;
    bool const finite = finite_matrix(q.coefficients()) && finite_matrix(measurement) && finite_matrix(reference);
    bool const nonzero =
        (measurement(0U) != Scalar{0} || measurement(1U) != Scalar{0} || measurement(2U) != Scalar{0}) &&
        (reference(0U) != Scalar{0} || reference(1U) != Scalar{0} || reference(2U) != Scalar{0});
    RotationCall::result = rotation;
    RotationCall::argument = q;
    RotationCall::count = 0U;
    auto const actual = formal_eskf::detail::prepare_magnetic_observation(q, measurement, reference);
    Status expected = !finite ? Status::non_finite_input : !nonzero ? Status::domain_error : Status::success;
    if (finite && nonzero)
    {
        __ESBMC_assert(same_matrix(actual.R, rotation),
                       "E-MAG-TILT: prepared rotation is exactly the prior producer result");
        for (std::size_t i = 0U; i < 3U; ++i)
        {
            Scalar predicted{0};
            for (std::size_t k = 0U; k < 3U; ++k)
                predicted = predicted + rotation(k, i) * reference(k);
            Scalar const residual = measurement(i) - predicted;
            __ESBMC_assert(same(actual.h_m_b(i), predicted) && same(actual.r(i), residual),
                           "E-MAG-TILT: prepared prediction and residual retain transpose, units and IEEE order");
            if (!std::isfinite(residual))
                expected = Status::non_finite_result;
        }
    }
    __ESBMC_assert(actual.status == expected && RotationCall::count == (finite && nonzero ? 1U : 0U),
                   "E-MAG-TILT: preparation validates all inputs and nonzero fields before rotation");
    __ESBMC_assert(same_matrix(q.coefficients(), old_q.coefficients()) && same_matrix(measurement, old_measurement) &&
                       same_matrix(reference, old_reference),
                   "E-MAG-TILT: preparation preserves all inputs");
}

void verify_magnetic_projector(Matrix3 rotation, Matrix3 output)
{
    auto const old_rotation = rotation;
    auto const before = output;
    formal_eskf::verification::ScalarArray<Scalar, 3U> down;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        down[i] = Scalar{0};
        for (std::size_t k = 0U; k < 3U; ++k)
            down[i] = down[i] + rotation(k, i) * (k == 2U ? Scalar{1} : Scalar{0});
    }
    Matrix3 expected;
    for (std::size_t i = 0U; i < 3U; ++i)
        for (std::size_t j = 0U; j < 3U; ++j)
            expected(i, j) = Scalar{0} + down[i] * down[j];
    bool const valid = finite_matrix(expected);
    auto const status = formal_eskf::detail::magnetic_tilt_projector(rotation, output);
    __ESBMC_assert(status == (valid ? Status::success : Status::non_finite_result),
                   "E-MAG-TILT: reject every nonfinite projector, including finite-input overflow");
    __ESBMC_assert((valid ? same_matrix(output, expected) : same_matrix(output, before)) &&
                       same_matrix(rotation, old_rotation),
                   "E-MAG-TILT: exact ordered Down outer product or atomic failure, with original R preserved");
    if (status == Status::success)
        for (std::size_t i = 0U; i < 3U; ++i)
            __ESBMC_assert(std::isfinite(down[i]),
                           "E-MAG-TILT: finite projector also establishes finite vertical axis");
}

void verify_magnetic_gain(Matrix3 projector, Gain gain)
{
    auto const old_projector = projector;
    auto const before = gain;
    Matrix3 expected;
    for (std::size_t i = 0U; i < 3U; ++i)
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            Scalar entry{0};
            for (std::size_t k = 0U; k < 3U; ++k)
                entry = entry + projector(i, k) * before(attitude_offset + k, j);
            expected(i, j) = entry;
        }
    bool const valid = finite_matrix(expected);
    auto const status = formal_eskf::detail::project_magnetic_gain<attitude_offset>(projector, gain);
    __ESBMC_assert(status == (valid ? Status::success : Status::non_finite_result),
                   "E-MAG-TILT: projected-gain arithmetic failure is detected");
    for (std::size_t i = 0U; i < state_size; ++i)
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            Scalar const value = valid && i >= attitude_offset && i < attitude_offset + 3U
                                     ? expected(i - attitude_offset, j)
                                     : before(i, j);
            __ESBMC_assert(same(gain(i, j), value),
                           "E-MAG-TILT: project only the attitude block, preserve other rows and rollback on failure");
        }
    __ESBMC_assert(same_matrix(projector, old_projector), "E-MAG-TILT: gain projection preserves its projector");
}
int main() { return 0; }
