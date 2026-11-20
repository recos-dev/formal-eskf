// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
namespace
{
using namespace output_boundary;
unsigned commits;
Sample committed;
} // namespace
namespace formal_eskf::px4
{
template <> void OutputPredictor<output_proof::Backend>::commit_history()
{
    check();
    __ESBMC_assert(this == owner && commits == 0 && m_history.m_count > 0,
                   "Output reset finish commits exactly one nonempty history");
    __ESBMC_init_object(owner);
    restore_outer();
    m_current = committed;
    expect_current(committed);
    // This caller permits arbitrary producer storage. reset_frame.cpp checks
    // every storage frame; the commit steps supply the actual copy equations.
    ++commits;
}
} // namespace formal_eskf::px4
#include <source_steps.hpp>
void verify_output_reset_plan(Output object, Quaternion q, Vector velocity, Vector position,
                              output_steps::ResetPlan plan)
{
    owner = &object;
    metadata(object);
    capture();
    auto const original_plan = plan;
    auto const input_q = q;
    auto const input_v = velocity;
    auto const input_p = position;
    auto const c = q.coefficients();
    Scalar norm{0};
    for (std::size_t i = 0; i < 4; ++i)
    {
        norm = norm + c(i) * c(i);
    }
    Scalar const residual = norm - Scalar{1};
    Scalar const tolerance = static_cast<Scalar>(profile::quaternion_squared_norm_tolerance);
    bool valid = residual >= -tolerance && residual <= tolerance;
    for (std::size_t i = 0; i < 3; ++i)
    {
        valid = std::isfinite(velocity(i)) & std::isfinite(position(i)) & valid;
    }
    bool const active = integers[field::time] != 0 && fault == Output::Fault::none;
    bool const result = output_steps::reset_plan(object, q, velocity, position, plan);
    __ESBMC_assert(result == (active && valid),
                   "Output reset prefix rejects inactive or invalid requests before staging");
    if (active && !valid)
    {
        fault = Output::Fault::invalid_state;
    }
    if (result)
    {
        __ESBMC_assert(plan.seed_only == (integers[field::count] == 0) &&
                           plan.count == (integers[field::count] == 0 ? 1 : integers[field::count]) &&
                           plan.index == integers[field::oldest],
                       "Output reset plans exactly the seed or all active physical records");
    }
    else
    {
        __ESBMC_assert(plan.seed_only == original_plan.seed_only && plan.count == original_plan.count &&
                           plan.index == original_plan.index,
                       "Rejected reset prefix preserves its local plan");
    }
    check();
    auto const old = input_q.coefficients();
    auto const after = q.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(old(i), after(i)), "Output reset prefix preserves its quaternion input");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(velocity(i), input_v(i)) && same(position(i), input_p(i)),
                       "Output reset prefix preserves translation inputs");
    }
}
void verify_output_reset_finish(Output object, Sample newest)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_FRAME_FIRST == 0 && FORMAL_ESKF_PROOF_FRAME_END == 0,
                   "Runner error: reset publication requires its complementary storage frame profiles");
    owner = &object;
    metadata(object);
    __ESBMC_assume(object.m_history.m_length >= 2);
    capture();
    bool const seed_only = integers[field::count] == 0;
    committed = newest;
    commits = 0;
    auto const expected_seed = object.m_staged[integers[field::oldest]];
    bool const result = output_steps::reset_finish(object, seed_only);
    __ESBMC_assert(result && commits == static_cast<unsigned>(!seed_only),
                   "Output reset publishes history or the seed without synthetic records");
    if (seed_only)
    {
        expect_current(expected_seed);
    }
    check();
}
