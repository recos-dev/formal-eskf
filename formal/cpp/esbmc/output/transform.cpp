// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
// Quaternion producers are pure except for the designated normalization output.
// Their summaries check all operands, output identity and call order; neither
// havocs the owner. The complete step postcondition checks every owner/history/
// workspace field, including every failure path. Intermediate full-owner copies
// are unnecessary premises for these pure producer contracts.

namespace
{
using namespace output_boundary;
Quaternion returned_q;
formal_eskf::Status returned_status;
Quaternion::vector4_type expected_coefficients;
unsigned normalizations, products;
Quaternion raw_product, expected_left, expected_right;
} // namespace
namespace formal_eskf::so3
{
template <>
constexpr UnitQuaternion<output_proof::Backend>
UnitQuaternion<output_proof::Backend>::operator*(UnitQuaternion const & other) const
{
    // This summary covers runtime calls. The unused constant-evaluation branch
    // retains a valid C++20 constexpr declaration; call counters check runtime use.
    if (__builtin_is_constant_evaluated())
    {
        return {};
    }
    __ESBMC_assert(products == 0 && normalizations == 0, "Output reset forms the left product before normalization");
    __ESBMC_assert(same(q0(), expected_left.q0()) && same(other.q0(), expected_right.q0()),
                   "Output Hamilton product complete operand order: coefficient 0");
    __ESBMC_assert(same(q1(), expected_left.q1()) && same(other.q1(), expected_right.q1()),
                   "Output Hamilton product complete operand order: coefficient 1");
    __ESBMC_assert(same(q2(), expected_left.q2()) && same(other.q2(), expected_right.q2()),
                   "Output Hamilton product complete operand order: coefficient 2");
    __ESBMC_assert(same(q3(), expected_left.q3()) && same(other.q3(), expected_right.q3()),
                   "Output Hamilton product complete operand order: coefficient 3");
    ++products;
    return raw_product;
}
template <>
Status UnitQuaternion<output_proof::Backend>::from_coefficients(vector4_type const & input, value_type minimum,
                                                                UnitQuaternion & result)
{
    __ESBMC_assert(normalizations == 0 && products == 1 && &result == &owner->m_staged[FORMAL_ESKF_PROOF_SLOT].q_nb,
                   "Output reset normalizes the current staged record exactly once");
    __ESBMC_assert(same(minimum, static_cast<Scalar>(profile::minimum_quaternion_norm)),
                   "Output reset forwards the profile normalization threshold");
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(input(i), expected_coefficients(i)),
                       "Output reset forwards every left Hamilton product coefficient");
    }
    if (returned_status == Status::success)
    {
        result = returned_q;
    }
    ++normalizations;
    return returned_status;
}
} // namespace formal_eskf::so3
#include <source_steps.hpp>
void verify_output_transform_step(Output object, Quaternion delta_q, Vector velocity, Vector position,
                                  Quaternion normalized, formal_eskf::Status status, Quaternion product)
{
    constexpr bool seed_only = FORMAL_ESKF_PROOF_SEED_ONLY != 0;
    constexpr std::size_t slot = FORMAL_ESKF_PROOF_SLOT;
    static_assert(slot < 64);
    owner = &object;
    metadata(object);
    __ESBMC_assume(slot < object.m_history.m_length);
    capture();
    auto const input_q = delta_q;
    auto const input_v = velocity;
    auto const input_p = position;
    Sample original;
    original.time_us = seed_only ? object.m_current.time_us : object.m_history.m_samples[slot].time_us;
    original.start_time_us =
        seed_only ? object.m_current.start_time_us : object.m_history.m_samples[slot].start_time_us;
    original.angular_period_us =
        seed_only ? object.m_current.angular_period_us : object.m_history.m_samples[slot].angular_period_us;
    original.q_nb.m_q0 = seed_only ? object.m_current.q_nb.q0() : object.m_history.m_samples[slot].q_nb.q0();
    original.q_nb.m_q1 = seed_only ? object.m_current.q_nb.q1() : object.m_history.m_samples[slot].q_nb.q1();
    original.q_nb.m_q2 = seed_only ? object.m_current.q_nb.q2() : object.m_history.m_samples[slot].q_nb.q2();
    original.q_nb.m_q3 = seed_only ? object.m_current.q_nb.q3() : object.m_history.m_samples[slot].q_nb.q3();
    for (std::size_t i = 0; i < 3; ++i)
    {
        original.v_n(i) = seed_only ? object.m_current.v_n(i) : object.m_history.m_samples[slot].v_n(i);
        original.p_n(i) = seed_only ? object.m_current.p_n(i) : object.m_history.m_samples[slot].p_n(i);
    }
    auto const b = original.q_nb.coefficients();
    expected_left = delta_q;
    expected_right = original.q_nb;
    raw_product = product;
    expected_coefficients = product.coefficients();
    products = 0;
    // The pre-normalization frame includes the copied and translated candidate.
    staged_integers[slot][0] = original.time_us;
    staged_integers[slot][1] = original.start_time_us;
    staged_integers[slot][2] = original.angular_period_us;
    for (std::size_t i = 0; i < 4; ++i)
    {
        staged_values[slot][i] = b(i);
    }
    bool finite = true;
    for (std::size_t i = 0; i < 3; ++i)
    {
        staged_values[slot][4 + i] = original.v_n(i) + velocity(i);
        staged_values[slot][7 + i] = original.p_n(i) + position(i);
        finite = std::isfinite(staged_values[slot][4 + i]) & std::isfinite(staged_values[slot][7 + i]) & finite;
    }
    returned_q = normalized;
    returned_status = status;
    normalizations = 0;
    auto index = slot;
    bool const result = output_steps::transform(object, index, seed_only, delta_q, velocity, position);
    bool const expected = status == formal_eskf::Status::success && finite;
    __ESBMC_assert(result == expected && normalizations == 1 && products == 1 &&
                       index == (expected ? (slot + 1 == integers[field::length] ? 0 : slot + 1) : slot),
                   "Output reset step propagates normalization/finite failures before cursor advancement");
    if (status == formal_eskf::Status::success)
    {
        auto const q = normalized.coefficients();
        for (std::size_t i = 0; i < 4; ++i)
        {
            staged_values[slot][i] = q(i);
        }
    }
    if (!expected)
    {
        fault = Output::Fault::numerical_failure;
    }
    check();
    auto const unchanged = delta_q.coefficients();
    auto const original_q = input_q.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(unchanged(i), original_q(i)), "Output reset step preserves the external attitude delta");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(velocity(i), input_v(i)) && same(position(i), input_p(i)),
                       "Output reset step preserves external translation deltas");
    }
}
