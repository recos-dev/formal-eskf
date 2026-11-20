// SPDX-License-Identifier: BSD-3-Clause

// Actual heading-reset caller and fuse_mag's reset/feedback suffix. The
// observation/lifecycle prefix is a separate contract; a failed feedback status
// must reach check() even if the main numerical reset has already committed.
#include "../contracts.hpp"
#include <array>
#include "../../../../integrations/px4/module/DeploymentProfile.hpp"

extern bool nondet_bool();
extern unsigned nondet_value();
extern std::uint64_t nondet_time();
extern double nondet_double();
extern void __ESBMC_init_object(void *);
#define PX4_ERR(...) ((void)0)

namespace
{
namespace profile = formal_eskf::px4::profile;
using Backend = contract_proof::Linalg;
using Vector = formal_eskf::linalg::Vector<Backend, 3>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
using Status = formal_eskf::Status;
using Action = formal_eskf::runtime::MagnetometerFusion::Action;
struct LifecycleState
{
    bool active{};
    Action action = Action::none;
};
struct Conditions
{
    bool horizontal_aiding{};
};
struct Event
{
    std::uint64_t time_us{};
};
struct Boundary
{
    Status core{}, feedback{};
    bool notification{}, active_after_feedback{}, core_called{}, feedback_called{}, notification_called{};
    unsigned * state{};
    unsigned * covariance{};
    Vector const * field{};
    Vector const * reference{};
    Quaternion delta;
    std::uint64_t event_time{};
} boundary;
} // namespace
namespace formal_eskf::runtime
{
Status reset_magnetic_heading(unsigned & state, unsigned & covariance, Vector const & field, Vector const & reference,
                              double variance, double minimum_horizontal, double minimum_norm, unsigned & output,
                              unsigned & output_covariance, Quaternion & delta)
{
    __ESBMC_assert(!boundary.core_called && &state == boundary.state && &covariance == boundary.covariance &&
                       &output == &state && &output_covariance == &covariance && &field == boundary.field &&
                       &reference == boundary.reference,
                   "Heading reset binds the main state/P and selected filtered field/reference");
    __ESBMC_assert(variance == profile::magnetic::heading_noise_rad * profile::magnetic::heading_noise_rad &&
                       minimum_horizontal == profile::magnetic::field.minimum_horizontal_field &&
                       minimum_norm == profile::minimum_quaternion_norm,
                   "Heading reset forwards the deployment numerical thresholds");
    boundary.core_called = true;
    delta = boundary.delta;
    return boundary.core;
}
} // namespace formal_eskf::runtime
namespace
{
struct Lifecycle
{
    using Action = formal_eskf::runtime::MagnetometerFusion::Action;
    LifecycleState value;
    LifecycleState const & state() const { return value; }
    Status record_reset(std::uint64_t time, bool reset)
    {
        __ESBMC_assert(boundary.core_called && !boundary.feedback_called && time == boundary.event_time &&
                           reset == formal_eskf::succeeded(boundary.core),
                       "Magnetic lifecycle receives the true numerical reset result at the observation epoch");
        boundary.feedback_called = true;
        value.active = boundary.active_after_feedback;
        return boundary.feedback;
    }
};
struct Ratio
{
    bool called{};
    void reset(std::uint64_t time)
    {
        __ESBMC_assert(boundary.notification_called && boundary.notification && time == boundary.event_time,
                       "Magnetic ratio reset follows accepted output notification");
        called = true;
    }
};
struct Heading
{
    bool reset{};
    void reset_heading() { reset = true; }
};
struct Flow
{
    using status_type = Status;
    using quaternion_type = Quaternion;
    using magnetic_lifecycle_type = Lifecycle;
    unsigned m_state{}, m_covariance{}, m_errors{}, m_rejections{};
    std::uint64_t m_time_us{};
    bool m_fault{}, m_heading_aligned{};
    struct Magnetic
    {
        struct Filter
        {
            Vector field;
            Vector const & field_b() const { return field; }
        } filter;
        Lifecycle lifecycle;
        Heading heading;
        std::array<Ratio, 3> ratios;
        std::uint32_t corrections{};
    } m_magnetic;
    struct Model
    {
        Vector reference;
        Vector const & observation_n() const { return reference; }
    } m_magnetic_model;
    struct Resets
    {
        Quaternion attitude_delta;
        std::uint8_t attitude_count{}, odometry_count{};
    } m_resets;
    bool check(Status status);
    bool reset_output(Quaternion const & q, Vector const & velocity, Vector const & position)
    {
        __ESBMC_assert(boundary.core_called && formal_eskf::succeeded(boundary.core) && boundary.feedback_called &&
                           formal_eskf::succeeded(boundary.feedback) &&
                           contract_proof::same_vector(q.coefficients(), boundary.delta.coefficients()) &&
                           contract_proof::same_vector(velocity, Vector{}) &&
                           contract_proof::same_vector(position, Vector{}),
                       "Magnetic reset sends the exact rotation delta and zero translations after successful feedback");
        boundary.notification_called = true;
        m_fault |= !boundary.notification;
        return boundary.notification;
    }
    Status reset_magnetic_heading(std::uint64_t sample_time_us);
    void finish_magnetic(bool fused, bool rejected, LifecycleState const & previous, Conditions const & conditions,
                         Event const & event);
};
#include "module_status_check.inc"
#include "module_magnetic_reset.inc"
#include "module_magnetic_finish.inc"

Status symbolic_status()
{
    auto const value = nondet_value();
    __ESBMC_assume(value <= unsigned(Status::non_finite_result));
    return static_cast<Status>(value);
}
} // namespace

extern "C" void verify_module_magnetic_reset()
{
    Flow flow;
    flow.m_errors = nondet_value();
    flow.m_rejections = nondet_value();
    flow.m_magnetic.corrections = nondet_value();
    flow.m_resets.attitude_count = static_cast<std::uint8_t>(nondet_value());
    flow.m_resets.odometry_count = static_cast<std::uint8_t>(nondet_value());
    auto const rejections = flow.m_rejections;
    auto const corrections = flow.m_magnetic.corrections;
    auto const attitude = flow.m_resets.attitude_count;
    auto const odometry = flow.m_resets.odometry_count;
    LifecycleState previous;
    previous.active = nondet_bool();
    Conditions conditions{nondet_bool()};
    Event event{nondet_time()};
    bool const selected = nondet_bool();
    bool const fused = nondet_bool();
    bool const rejected = nondet_bool();
    flow.m_magnetic.lifecycle.value.action = selected ? Action::reset : Action::none;
    flow.m_magnetic.lifecycle.value.active = nondet_bool();
    flow.m_heading_aligned = nondet_bool();
    flow.m_time_us = nondet_time();
    boundary.state = &flow.m_state;
    boundary.covariance = &flow.m_covariance;
    boundary.field = &flow.m_magnetic.filter.field;
    boundary.reference = &flow.m_magnetic_model.reference;
    boundary.event_time = event.time_us;
    boundary.core = symbolic_status();
    boundary.feedback = symbolic_status();
    boundary.notification = nondet_bool();
    boundary.active_after_feedback = nondet_bool();
    Quaternion delta;
    __ESBMC_init_object(&delta);
    boundary.delta = delta;
    bool const aligned = flow.m_heading_aligned;
    flow.finish_magnetic(fused, rejected, previous, conditions, event);
    bool const notified =
        selected && formal_eskf::succeeded(boundary.core) && formal_eskf::succeeded(boundary.feedback);
    bool const reset = notified && boundary.notification;
    bool const failed = selected && !reset;
    bool const stopped = previous.active && !flow.m_magnetic.lifecycle.value.active && !conditions.horizontal_aiding;
    __ESBMC_assert(boundary.core_called == selected && boundary.feedback_called == selected &&
                       boundary.notification_called == notified && flow.m_fault == failed,
                   "Core reset, lifecycle feedback and output failure all reach the enclosing magnetic fault gate");
    __ESBMC_assert(flow.m_magnetic.heading.reset == reset &&
                       flow.m_resets.attitude_count == static_cast<std::uint8_t>(attitude + unsigned(reset)) &&
                       flow.m_resets.odometry_count == static_cast<std::uint8_t>(odometry + unsigned(reset)) &&
                       flow.m_magnetic.corrections == corrections + unsigned(!failed && fused) &&
                       flow.m_rejections == rejections + unsigned(!failed && rejected),
                   "Magnetic reset and fusion accounting are committed only after the reset status is accepted");
    __ESBMC_assert(flow.m_heading_aligned == (stopped ? false : (reset || aligned)),
                   "Post-feedback lifecycle stop revokes alignment using the current mutable owner state");
    if (reset)
    {
        __ESBMC_assert(
            contract_proof::same_vector(flow.m_resets.attitude_delta.coefficients(), boundary.delta.coefficients()),
            "Heading report preserves the exact output-history rotation delta");
    }
}
