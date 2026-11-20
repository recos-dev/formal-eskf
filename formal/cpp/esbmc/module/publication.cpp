// SPDX-License-Identifier: BSD-3-Clause

// Actual navigation-publication gate and exact message-epoch prefixes. Payload
// serialization, geographic projection and uORB delivery use native evidence;
// the reviewed source frame excludes later timestamp writes/aliases.
#include <cstdint>
#include <formal_eskf/status.hpp>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);
#define PX4_ERR(...) ((void)0)

namespace
{
#include "module_constants.inc"
struct Vector
{
    bool finite{};
};
struct Message
{
    std::uint64_t timestamp{}, timestamp_sample{};
};
using vehicle_attitude_s = Message;
using vehicle_local_position_s = Message;
using vehicle_global_position_s = Message;
using vehicle_odometry_s = Message;
using estimator_status_s = Message;
using estimator_status_flags_s = Message;
using estimator_sensor_bias_s = Message;
std::uint64_t clock_time;
std::uint64_t hrt_absolute_time() { return clock_time; }
} // namespace
namespace formal_eskf::linalg
{
bool all_finite(Vector vector) { return vector.finite; }
} // namespace formal_eskf::linalg
namespace
{
struct Flow
{
    using imu_type = unsigned;
    using status_type = formal_eskf::Status;
    struct OutputValidity
    {
        bool healthy{};
    } validity;
    struct BodyOutput
    {
        Vector position_n{}, velocity_n{};
    } body;
    struct Output
    {
        struct Sample
        {
            std::uint64_t time_us{};
        } sample;
        Sample const & current() const { return sample; }
    } m_output;
    std::uint64_t m_time_us{}, m_last_output{};
    bool m_fault{}, validity_called{}, body_called{};
    unsigned m_errors{}, publications{};
    OutputValidity output_validity(std::uint64_t now)
    {
        __ESBMC_assert(now == clock_time && publications == 0, "Navigation validity is checked before publication");
        validity_called = true;
        return validity;
    }
    BodyOutput body_output()
    {
        __ESBMC_assert(validity_called && validity.healthy, "Body conversion follows a healthy output gate");
        body_called = true;
        return body;
    }
    Message publish_local_position(std::uint64_t now, unsigned const & imu, BodyOutput const & value,
                                   OutputValidity const & result)
    {
        __ESBMC_assert(publications == 0 && body_called && value.position_n.finite && value.velocity_n.finite &&
                           result.healthy && !m_fault && m_last_output == m_output.current().time_us && imu == 1,
                       "Local publication follows finite body conversion and output watermark update");
        ++publications;
        return local_epochs(now);
    }
    void publish_global_position(std::uint64_t now, Message const & local, BodyOutput const &, OutputValidity const &)
    {
        __ESBMC_assert(publications == 1 && local.timestamp == now &&
                           local.timestamp_sample == m_output.current().time_us,
                       "Global publication receives the same current local output and epoch");
        ++publications;
    }
    void publish_odometry(std::uint64_t now, unsigned const & imu, BodyOutput const &)
    {
        __ESBMC_assert(publications == 2 && now == clock_time && imu == 1,
                       "Odometry completes the same current-output publication group");
        ++publications;
    }
    bool check(status_type status);
    void publish(unsigned const & imu);
    Message attitude_epochs(std::uint64_t now) const;
    Message local_epochs(std::uint64_t now) const;
    Message global_epochs(std::uint64_t now) const;
    Message odometry_epochs(std::uint64_t now) const;
    Message status_epochs(std::uint64_t now) const;
    Message flags_epochs(std::uint64_t now) const;
    Message bias_epochs(std::uint64_t now) const;
};
#include "module_status_check.inc"
#include "module_publication.inc"
#include "module_epochs.inc"
} // namespace

extern "C" void verify_module_publication()
{
    Flow flow;
    flow.m_time_us = nondet_time();
    flow.m_output.sample.time_us = nondet_time();
    flow.m_last_output = nondet_time();
    flow.m_errors = nondet_value();
    flow.m_fault = nondet_bool();
    flow.validity.healthy = nondet_bool();
    flow.body.position_n.finite = nondet_bool();
    flow.body.velocity_n.finite = nondet_bool();
    clock_time = nondet_time();
    // Accepted output-owner monotonicity and the separately proved validity gate.
    __ESBMC_assume(flow.m_last_output <= flow.m_output.current().time_us);
    __ESBMC_assume(!flow.validity.healthy || !flow.m_fault);
    bool const fault = flow.m_fault;
    auto const previous = flow.m_last_output;
    auto const errors = flow.m_errors;
    bool const due =
        flow.validity.healthy && flow.m_output.current().time_us - previous >= profile::output::navigation_interval_us;
    bool const finite = flow.body.position_n.finite && flow.body.velocity_n.finite;
    flow.publish(1);
    __ESBMC_assert(flow.validity_called && flow.body_called == due && flow.publications == (due && finite ? 3U : 0U),
                   "Navigation publishes all three topics only at an eligible finite current output");
    __ESBMC_assert(flow.m_last_output == (due && finite ? flow.m_output.current().time_us : previous) &&
                       flow.m_fault == (fault || (due && !finite)) &&
                       flow.m_errors == errors + unsigned(due && !finite),
                   "Conversion failure latches fault and preserves publication watermark");
}

extern "C" void verify_module_message_epochs()
{
    Flow flow;
    flow.m_time_us = nondet_time();
    flow.m_output.sample.time_us = nondet_time();
    auto const now = nondet_time();
    auto const attitude = flow.attitude_epochs(now);
    auto const local = flow.local_epochs(now);
    auto const global = flow.global_epochs(now);
    auto const odometry = flow.odometry_epochs(now);
    auto const status = flow.status_epochs(now);
    auto const flags = flow.flags_epochs(now);
    auto const bias = flow.bias_epochs(now);
    __ESBMC_assert(attitude.timestamp_sample == flow.m_output.current().time_us &&
                       local.timestamp_sample == flow.m_output.current().time_us &&
                       global.timestamp_sample == flow.m_output.current().time_us &&
                       odometry.timestamp_sample == flow.m_output.current().time_us,
                   "All four navigation/attitude topics carry the current output epoch");
    __ESBMC_assert(status.timestamp_sample == flow.m_time_us && flags.timestamp_sample == flow.m_time_us &&
                       bias.timestamp_sample == flow.m_time_us,
                   "Estimator status, flags and bias carry the delayed main epoch");
    __ESBMC_assert(attitude.timestamp == now && local.timestamp == now && global.timestamp == now &&
                       odometry.timestamp == now && status.timestamp == now && flags.timestamp == now &&
                       bias.timestamp == now,
                   "All seven topic timestamps retain the caller's publication time");
}
