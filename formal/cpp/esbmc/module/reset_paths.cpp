// SPDX-License-Identifier: BSD-3-Clause

// Each actual selected navigation-reset branch: numerical reset, lifecycle
// feedback, output notification, then reporting. Branch selection belongs to
// the lifecycle/height contracts; these checks do not rewrite those policies.
#include "../contracts.hpp"
#include <array>
#include <formal_eskf/runtime/gnss_lifecycle.hpp>

extern bool nondet_bool();
extern double nondet_double();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
#define PX4_ERR(...) ((void)0)

namespace
{
using Backend = contract_proof::Linalg;
using Vector = formal_eskf::linalg::Vector<Backend, 3>;
using Vector2 = formal_eskf::linalg::Vector<Backend, 2>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
using Status = formal_eskf::Status;
using Group = formal_eskf::runtime::GnssFusion::Group;
static_assert(sizeof(Backend::value_type) == sizeof(double));
enum class Kind
{
    velocity,
    vertical_velocity,
    position
};
struct Boundary
{
    Kind kind{};
    Status core{}, feedback{};
    bool notification{}, reset_called{}, feedback_called{}, notification_called{};
    unsigned * state{};
    unsigned * covariance{};
    Vector velocity, variances, delta;
    Vector2 position;
    double position_variance{};
    std::uint64_t sample_time{};
} boundary;

Vector vector3(double x, double y, double z) { return Vector::from_row_major({x, y, z}); }
void reset_frame(unsigned & state, unsigned & covariance, unsigned & output, unsigned & output_covariance)
{
    __ESBMC_assert(!boundary.reset_called && &state == boundary.state && &covariance == boundary.covariance &&
                       &output == &state && &output_covariance == &covariance,
                   "Selected reset operates in place on the actual main state/covariance pair");
    boundary.reset_called = true;
}
} // namespace
namespace formal_eskf::runtime
{
Status reset_velocity(unsigned & state, unsigned & covariance, Vector const & value, Vector const & variance,
                      unsigned & output, unsigned & output_covariance, Vector & delta)
{
    reset_frame(state, covariance, output, output_covariance);
    __ESBMC_assert(boundary.kind == Kind::velocity && contract_proof::same_vector(value, boundary.velocity) &&
                       contract_proof::same_vector(variance, boundary.variances),
                   "Full velocity reset forwards its selected observation and diagonal variances");
    delta = boundary.delta;
    return boundary.core;
}
Status reset_vertical_velocity(unsigned & state, unsigned & covariance, double value, double variance,
                               unsigned & output, unsigned & output_covariance, double & delta)
{
    reset_frame(state, covariance, output, output_covariance);
    __ESBMC_assert(boundary.kind == Kind::vertical_velocity && contract_proof::same(value, boundary.velocity(2)) &&
                       contract_proof::same(variance, boundary.variances(2)),
                   "Down-only reset forwards the vertical observation and variance");
    delta = boundary.delta(2);
    return boundary.core;
}
Status reset_gnss_horizontal_position(unsigned & state, unsigned & covariance, Vector2 const & value,
                                      Vector2 const & variance, unsigned & output, unsigned & output_covariance,
                                      Vector2 & delta)
{
    reset_frame(state, covariance, output, output_covariance);
    __ESBMC_assert(boundary.kind == Kind::position && contract_proof::same_vector(value, boundary.position) &&
                       contract_proof::same(variance(0), boundary.position_variance) &&
                       contract_proof::same(variance(1), boundary.position_variance),
                   "Horizontal reset forwards selected position with equal configured horizontal variances");
    delta = Vector2::from_row_major({boundary.delta(0), boundary.delta(1)});
    return boundary.core;
}
} // namespace formal_eskf::runtime
namespace
{
struct Lifecycle
{
    using Group = formal_eskf::runtime::GnssFusion::Group;
    Status record_reset(Group group, std::uint64_t time, bool success)
    {
        __ESBMC_assert(boundary.reset_called && !boundary.feedback_called && time == boundary.sample_time &&
                           success == formal_eskf::succeeded(boundary.core) &&
                           group == (boundary.kind == Kind::velocity ? Group::velocity : Group::horizontal_position),
                       "Lifecycle feedback records the actual numerical reset outcome at the selected source epoch");
        boundary.feedback_called = true;
        return boundary.feedback;
    }
};
struct Ratio
{
    bool reset_called{};
    void reset(std::uint64_t time)
    {
        __ESBMC_assert(boundary.notification_called && boundary.notification && time == boundary.sample_time,
                       "Diagnostic ratio resets follow successful output notification at the source epoch");
        reset_called = true;
    }
};
struct Reporting
{
    unsigned calls{};
    std::uint8_t odometry_count{};
    std::uint64_t expected_epoch{};
    void record_velocity(std::uint64_t time, Vector const & delta)
    {
        __ESBMC_assert(boundary.notification_called && boundary.notification && time == expected_epoch &&
                           contract_proof::same_vector(delta, boundary.delta),
                       "Velocity reset is reported only after output history accepted the same delta");
        ++calls;
    }
    void record_vertical_velocity(std::uint64_t time, double delta)
    {
        __ESBMC_assert(boundary.notification_called && boundary.notification && time == expected_epoch &&
                           contract_proof::same(delta, boundary.delta(2)),
                       "Down velocity report retains the reset delta and main epoch");
        ++calls;
    }
    void record_position(std::uint64_t time, Vector2 const & delta)
    {
        __ESBMC_assert(boundary.notification_called && boundary.notification && time == expected_epoch &&
                           contract_proof::same(delta(0), boundary.delta(0)) &&
                           contract_proof::same(delta(1), boundary.delta(1)),
                       "Horizontal reset reporting follows successful output notification");
        ++calls;
    }
};
struct Flow
{
    using status_type = Status;
    using gnss_lifecycle_type = Lifecycle;
    using vector3_type = Vector;
    using vector2_type = Vector2;
    unsigned m_state{}, m_covariance{}, m_errors{};
    std::uint64_t m_time_us{};
    bool m_fault{};
    struct Gnss
    {
        struct Sample
        {
            std::uint64_t time_us{};
        } sample;
        struct Innovation
        {
            bool active{};
            double innovation{}, variance{};
        } vertical_innovation;
        Lifecycle lifecycle;
        std::array<Ratio, 3> velocity_ratios;
        std::array<Ratio, 2> position_ratios;
        Vector velocity_test_ratios;
        double position_test_ratio{};
        bool velocity_rejected{}, position_rejected{};
    } m_gnss;
    Reporting m_resets;
    bool check(Status status);
    bool reset_output(Quaternion const & q, Vector const & velocity, Vector const & position)
    {
        __ESBMC_assert(boundary.reset_called && formal_eskf::succeeded(boundary.core) &&
                           (boundary.kind == Kind::vertical_velocity ||
                            (boundary.feedback_called && formal_eskf::succeeded(boundary.feedback))),
                       "Output notification requires successful numerical reset and applicable lifecycle feedback");
        __ESBMC_assert(q.q0() == 1. && q.q1() == 0. && q.q2() == 0. && q.q3() == 0.,
                       "Navigation resets do not invent a rotation delta");
        Vector expected_velocity;
        Vector expected_position;
        if (boundary.kind == Kind::velocity)
        {
            expected_velocity = boundary.delta;
        }
        else if (boundary.kind == Kind::vertical_velocity)
        {
            expected_velocity = vector3(0., 0., boundary.delta(2));
        }
        else
        {
            expected_position = vector3(boundary.delta(0), boundary.delta(1), 0.);
        }
        __ESBMC_assert(contract_proof::same_vector(velocity, expected_velocity) &&
                           contract_proof::same_vector(position, expected_position),
                       "Output notification carries exactly the reset axes and deltas");
        boundary.notification_called = true;
        m_fault |= !boundary.notification;
        return boundary.notification;
    }
    bool velocity_reset_path(Vector const & velocity, Vector const & velocity_variances);
    bool vertical_velocity_reset_path(Vector const & velocity, Vector const & velocity_variances);
    bool position_reset_path(Vector2 const & xy, double position_variance);
};
#include "module_status_check.inc"
#include "module_reset_paths.inc"

Status symbolic_status()
{
    auto const value = nondet_value();
    __ESBMC_assume(value <= unsigned(Status::non_finite_result));
    return static_cast<Status>(value);
}
Vector symbolic_vector() { return vector3(nondet_double(), nondet_double(), nondet_double()); }
void verify_reset_path(Kind kind)
{
    Flow flow;
    flow.m_errors = nondet_value();
    flow.m_resets.odometry_count = static_cast<std::uint8_t>(nondet_value());
    auto const odometry = flow.m_resets.odometry_count;
    boundary.kind = kind;
    boundary.core = symbolic_status();
    boundary.feedback = symbolic_status();
    boundary.notification = nondet_bool();
    boundary.state = &flow.m_state;
    boundary.covariance = &flow.m_covariance;
    boundary.velocity = symbolic_vector();
    boundary.variances = symbolic_vector();
    boundary.delta = symbolic_vector();
    boundary.position = Vector2::from_row_major({nondet_double(), nondet_double()});
    boundary.position_variance = nondet_double();
    flow.m_time_us = nondet_time();
    flow.m_resets.expected_epoch = flow.m_time_us;
    flow.m_gnss.sample.time_us = boundary.sample_time = nondet_time();
    bool result{};
    if (kind == Kind::position)
    {
        result = flow.position_reset_path(boundary.position, boundary.position_variance);
    }
    else if (kind == Kind::velocity)
    {
        result = flow.velocity_reset_path(boundary.velocity, boundary.variances);
    }
    else
    {
        result = flow.vertical_velocity_reset_path(boundary.velocity, boundary.variances);
    }
    bool const numerical = formal_eskf::succeeded(boundary.core);
    bool const lifecycle = kind == Kind::vertical_velocity || formal_eskf::succeeded(boundary.feedback);
    bool const success = numerical && lifecycle && boundary.notification;
    __ESBMC_assert(result == success && flow.m_fault == !success && boundary.reset_called &&
                       boundary.notification_called == (numerical && lifecycle) &&
                       boundary.feedback_called == (kind != Kind::vertical_velocity),
                   "Every selected reset either notifies output successfully or returns with a latched failure");
    __ESBMC_assert(flow.m_resets.calls == (success ? (kind == Kind::velocity ? 2U : 1U) : 0U) &&
                       flow.m_resets.odometry_count == static_cast<std::uint8_t>(odometry + unsigned(success)),
                   "Reset counters/deltas are published only after successful output notification");
}
} // namespace
extern "C" void verify_module_velocity_reset() { verify_reset_path(Kind::velocity); }
extern "C" void verify_module_vertical_reset() { verify_reset_path(Kind::vertical_velocity); }
extern "C" void verify_module_position_reset() { verify_reset_path(Kind::position); }
