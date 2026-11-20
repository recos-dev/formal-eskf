// SPDX-License-Identifier: BSD-3-Clause

// Actual reset notification and body-point reporting, with the actual fixed
// matrix/quaternion types. Output reset and body conversion are separate
// producer contracts; arbitrary before/after values include every numeric case.
#include "../contracts.hpp"
#include <cstdint>
#include <initializer_list>

extern bool nondet_bool();
extern double nondet_double();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern void __ESBMC_init_object(void *);
#define PX4_ERR(...) ((void)0)

namespace
{
using Backend = contract_proof::Linalg;
using Vector = formal_eskf::linalg::Vector<Backend, 3>;
using Vector2 = formal_eskf::linalg::Vector<Backend, 2>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
static_assert(sizeof(Backend::value_type) == sizeof(double));
bool same(Vector const & a, Vector const & b) { return contract_proof::same_vector(a, b); }
bool same(Quaternion const & a, Quaternion const & b)
{
    return contract_proof::same_vector(a.coefficients(), b.coefficients());
}
struct Body
{
    Vector position_n{}, velocity_n{};
};
struct Output
{
    struct Sample
    {
        std::uint64_t time_us{};
    } sample;
    bool called{}, success{};
    Quaternion const * rotation{};
    Vector const * velocity{};
    Vector const * position{};
    Sample const & current() const { return sample; }
    bool apply_reset(Quaternion const & q, Vector const & v, Vector const & p)
    {
        __ESBMC_assert(!called && &q == rotation && &v == velocity && &p == position,
                       "Output reset receives the exact caller deltas, without changing their representation");
        called = true;
        return success;
    }
};
struct Reporting
{
    unsigned calls{};
    std::uint64_t expected_epoch{};
    Vector2 position;
    Vector velocity;
    double height{}, vertical_velocity{};
    void record_position(std::uint64_t time, Vector2 const & value)
    {
        __ESBMC_assert(calls == 0 && time == expected_epoch, "Body-point XY report uses the delayed reset epoch");
        ++calls;
        position = value;
    }
    void record_height(std::uint64_t time, double value, std::uint8_t count)
    {
        __ESBMC_assert(calls == 1 && time == expected_epoch && count == 1, "Body-point height reset is counted once");
        ++calls;
        height = value;
    }
    void record_velocity(std::uint64_t time, Vector const & value)
    {
        __ESBMC_assert(calls == 2 && time == expected_epoch, "Body-point velocity report follows position reports");
        ++calls;
        velocity = value;
    }
    void record_vertical_velocity(std::uint64_t time, double value)
    {
        __ESBMC_assert(calls == 3 && time == expected_epoch, "Body-point Down velocity uses the same reset epoch");
        ++calls;
        vertical_velocity = value;
    }
};
struct Flow
{
    using quaternion_type = Quaternion;
    using vector3_type = Vector;
    using vector2_type = Vector2;
    Output m_output;
    Reporting m_resets;
    bool m_fault{};
    unsigned m_errors{}, conversions{};
    std::uint64_t m_time_us{};
    struct Positions
    {
        Vector imu_b;
    } m_sensor_positions;
    Body before, after;
    Body body_output()
    {
        ++conversions;
        if (!m_output.called)
        {
            __ESBMC_assert(conversions == 1, "First body conversion precedes output reset");
            return before;
        }
        __ESBMC_assert(m_output.success && conversions == 2, "Second body conversion requires successful output reset");
        return after;
    }
    bool check_timing(bool succeeded);
    bool reset_output(Quaternion const & q, Vector const & v, Vector const & p);
};
#include "module_reset.inc"

Vector symbolic_vector() { return Vector::from_row_major({nondet_double(), nondet_double(), nondet_double()}); }
} // namespace

extern "C" void verify_module_reset()
{
    Flow flow;
    // No unit/finite restriction is needed for this caller control/reporting
    // claim. The output owner validates its own reset inputs independently.
    Quaternion q;
    __ESBMC_init_object(&q);
    auto const velocity = symbolic_vector();
    auto const position = symbolic_vector();
    flow.before = {symbolic_vector(), symbolic_vector()};
    flow.after = {symbolic_vector(), symbolic_vector()};
    flow.m_sensor_positions.imu_b = symbolic_vector();
    flow.m_output.sample.time_us = nondet_time();
    flow.m_output.success = nondet_bool();
    flow.m_output.rotation = &q;
    flow.m_output.velocity = &velocity;
    flow.m_output.position = &position;
    flow.m_fault = nondet_bool();
    flow.m_errors = nondet_value();
    flow.m_time_us = nondet_time();
    flow.m_resets.expected_epoch = flow.m_time_us;
    auto const epoch = flow.m_output.current().time_us;
    auto const errors = flow.m_errors;
    bool const fault = flow.m_fault;
    bool const ok = flow.reset_output(q, velocity, position);
    bool const seeded = epoch != 0;
    bool const point = q.q1() != 0. || q.q2() != 0. || q.q3() != 0.;
    bool const offset = formal_eskf::linalg::squared_norm(flow.m_sensor_positions.imu_b) > 0.;
    bool const reporting = seeded && flow.m_output.success && point && offset;
    __ESBMC_assert(ok == (!seeded || flow.m_output.success) && flow.m_output.called == seeded &&
                       flow.m_fault == (fault || (seeded && !flow.m_output.success)) &&
                       flow.m_errors == errors + unsigned(seeded && !flow.m_output.success),
                   "Unseeded startup needs no notification; seeded output failure is latched");
    __ESBMC_assert(flow.m_output.current().time_us == epoch && flow.m_resets.calls == (reporting ? 4U : 0U) &&
                       flow.conversions == unsigned(seeded) + unsigned(reporting),
                   "Reset preserves output epoch and reports body-point motion only after successful rotation reset");
    if (reporting)
    {
        auto const dp = flow.after.position_n - flow.before.position_n - position;
        auto const dv = flow.after.velocity_n - flow.before.velocity_n - velocity;
        __ESBMC_assert(contract_proof::same(flow.m_resets.position(0), dp(0)) &&
                           contract_proof::same(flow.m_resets.position(1), dp(1)) &&
                           contract_proof::same(flow.m_resets.height, dp(2)) && same(flow.m_resets.velocity, dv) &&
                           contract_proof::same(flow.m_resets.vertical_velocity, dv(2)),
                       "Body-point reports exclude translation deltas already reported by their callers");
    }
}
