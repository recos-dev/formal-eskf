// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
namespace
{
using namespace output_boundary;
State requested;
Quaternion raw_product, normalized;
Quaternion expected_left, expected_right;
Quaternion * expected_output;
formal_eskf::Status status_result;
bool valid;
unsigned validations, products, normalizations;
void check_state(State const & state)
{
    __ESBMC_assert(same(state.q_nb.q0(), requested.q_nb.q0()) && same(state.q_nb.q1(), requested.q_nb.q1()) &&
                       same(state.q_nb.q2(), requested.q_nb.q2()) && same(state.q_nb.q3(), requested.q_nb.q3()),
                   "Output feedback preserves all main quaternion coefficients");
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(state.v_n(i), requested.v_n(i)) && same(state.p_n(i), requested.p_n(i)) &&
                           same(state.b_a(i), requested.b_a(i)) && same(state.b_g(i), requested.b_g(i)),
                       "Output feedback preserves all main vectors and biases");
    }
}
} // namespace
namespace formal_eskf::px4
{
template <> bool OutputPredictor<output_proof::Backend>::valid_state(state_type const & state)
{
    check_state(state);
    __ESBMC_assert(validations == 0, "Output feedback admission validates main state exactly once");
    ++validations;
    return valid;
}
} // namespace formal_eskf::px4
namespace formal_eskf::so3
{
template <>
constexpr UnitQuaternion<output_proof::Backend>
UnitQuaternion<output_proof::Backend>::operator*(UnitQuaternion const & other) const
{
    if (__builtin_is_constant_evaluated())
    {
        return {};
    }
    __ESBMC_assert(products == 0 && normalizations == 0, "Output feedback forms its product before normalization");
    __ESBMC_assert(same(q0(), expected_left.q0()) && same(q1(), expected_left.q1()) && same(q2(), expected_left.q2()) &&
                       same(q3(), expected_left.q3()),
                   "Output feedback left operand is the conjugate of the main quaternion");
    __ESBMC_assert(same(other.q0(), expected_right.q0()) && same(other.q1(), expected_right.q1()) &&
                       same(other.q2(), expected_right.q2()) && same(other.q3(), expected_right.q3()),
                   "Output feedback right operand is the delayed quaternion");
    ++products;
    return raw_product;
}
template <>
Status UnitQuaternion<output_proof::Backend>::from_coefficients(vector4_type const & input, value_type minimum,
                                                                UnitQuaternion & output)
{
    __ESBMC_assert(products == 1 && normalizations == 0 && &output == expected_output,
                   "Output feedback normalizes the formed product into its designated local output once");
    auto const expected = raw_product.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(input(i), expected(i)), "Output feedback forwards each raw product coefficient");
    }
    __ESBMC_assert(same(minimum, static_cast<Scalar>(profile::minimum_quaternion_norm)),
                   "Output feedback uses the configured normalization threshold");
    ++normalizations;
    if (status_result == Status::success)
    {
        output = normalized;
    }
    return status_result;
}
} // namespace formal_eskf::so3
#include <feedback_steps.hpp>
void verify_output_feedback_admission(Output object, State state, std::uint64_t time, bool state_valid)
{
    owner = &object;
    metadata(object);
    capture();
    requested = state;
    valid = state_valid;
    validations = 0;
    bool const active = integers[field::time] != 0 && fault == Output::Fault::none;
    bool const full = integers[field::length] > 0 && integers[field::count] == integers[field::length];
    bool epoch = false;
    if (active && full)
    {
        auto const & delayed = object.m_history.m_samples[integers[field::oldest]];
        epoch = delayed.time_us > integers[field::fusion_time] && delayed.time_us == time &&
                delayed.start_time_us == integers[field::fusion_time];
    }
    Sample const * selected = nullptr;
    bool const result = output_steps::correction_plan(object, time, state, selected);
    __ESBMC_assert(result == (active && epoch && valid) && validations == static_cast<unsigned>(active && epoch),
                   "Output feedback exact activity, delayed-epoch and validation precedence");
    if (active && !epoch)
    {
        fault = Output::Fault::epoch_mismatch;
    }
    else if (active && !valid)
    {
        fault = Output::Fault::invalid_state;
    }
    if (result)
    {
        __ESBMC_assert(selected == &object.m_history.m_samples[integers[field::oldest]],
                       "Output feedback uses the original oldest history record");
    }
    check();
    check_state(state);
}
void verify_output_feedback_normalization(Output object, State state, Sample sample, Quaternion output,
                                          Quaternion product, Quaternion result, formal_eskf::Status status)
{
    owner = &object;
    capture();
    requested = state;
    auto const before_sample = sample;
    auto const before_output = output;
    expected_left.m_q0 = state.q_nb.q0();
    expected_left.m_q1 = -state.q_nb.q1();
    expected_left.m_q2 = -state.q_nb.q2();
    expected_left.m_q3 = -state.q_nb.q3();
    expected_right = sample.q_nb;
    raw_product = product;
    normalized = result;
    status_result = status;
    expected_output = &output;
    products = normalizations = 0;
    bool const accepted = output_steps::correction_normalize(object, state, &sample, output);
    __ESBMC_assert(accepted == (status == formal_eskf::Status::success) && products == 1 && normalizations == 1,
                   "Output feedback propagates normalization status after one ordered product");
    auto const after = output.coefficients();
    auto const old = before_output.coefficients();
    auto const value = result.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(after(i), accepted ? value(i) : old(i)),
                       "Output feedback publishes normalized quaternion or retains its prior local value");
    }
    if (!accepted)
    {
        fault = Output::Fault::numerical_failure;
    }
    check();
    check_state(state);
    __ESBMC_assert(same_sample(sample, before_sample), "Output feedback normalization preserves the delayed record");
}
