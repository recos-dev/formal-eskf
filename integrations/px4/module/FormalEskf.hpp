// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <mutex>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/linalg/backend/px4_matrix.hpp>

// Single writer: all filter state is owned by this work item. The core has no
// PX4 dependencies except its explicitly selected matrix backend.
class FormalEskf final : public ModuleBase<FormalEskf>, public px4::ScheduledWorkItem
{
public:
    FormalEskf();
    ~FormalEskf() override;
    static int task_spawn(int argc, char * argv[]);
    static int custom_command(int argc, char * argv[]);
    static int print_usage(char const * reason = nullptr);
    int print_status() override;
    bool init();

private:
    using backend_type = formal_eskf::linalg::Px4MatrixBackend<double>;
    using types_type = formal_eskf::EskfTypes<backend_type, formal_eskf::configuration::Ins>;
    using state_type = types_type::nominal_state_type;
    using covariance_type = types_type::error_covariance_type;
    using vector3_type = backend_type::vector_type<3>;
    using matrix3_type = backend_type::matrix_type<3, 3>;
    using imu_type = types_type::imu_sample_type;
    using status_type = formal_eskf::Status;

    void Run() override;
    void initialize(vehicle_imu_s const & sample, imu_type const & imu);
    void fuse_gps();
    void fuse_mag();
    void fuse_baro();
    void publish(imu_type const & imu);
    void publish_status(uint64_t now);
    bool gps_good() const;
    bool check(status_type status);
    bool recent(uint64_t sample, uint64_t max_age = 1000000) const;
    static vector3_type vector3(double x, double y, double z) { return vector3_type::from_row_major({x, y, z}); }
    static double square(double x) { return x * x; }
    static constexpr double minimum_quaternion_norm = 1e-6;

    // Axis-wise 5-sigma gate including projected prior covariance. This is a
    // conservative per-axis gate, not a full chi-square/NIS test.
    template <size_t N>
    double gate(backend_type::vector_type<N> const & residual, const backend_type::matrix_type<N, 15> & H,
                const backend_type::matrix_type<N, N> & R) const
    {
        const auto S = formal_eskf::linalg::sandwich(H, m_covariance) + R;
        double ratio = 0.;
        for (size_t i = 0; i < N; ++i)
        {
            if (!std::isfinite(residual(i)) || !std::isfinite(S(i, i)) || S(i, i) <= 0.)
            {
                return INFINITY;
            }
            ratio = std::max(ratio, square(residual(i)) / (25. * S(i, i)));
        }
        return ratio;
    }

    uORB::SubscriptionCallbackWorkItem m_imu_sub{this, ORB_ID(vehicle_imu)};
    uORB::Subscription m_gps_sub{ORB_ID(vehicle_gps_position)};
    uORB::Subscription m_mag_sub{ORB_ID(vehicle_magnetometer)};
    uORB::Subscription m_baro_sub{ORB_ID(vehicle_air_data)};
    uORB::Subscription m_land_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription m_vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Publication<vehicle_attitude_s> m_attitude_pub{ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_local_position_s> m_local_pub{ORB_ID(vehicle_local_position)};
    uORB::Publication<vehicle_global_position_s> m_global_pub{ORB_ID(vehicle_global_position)};
    uORB::Publication<vehicle_odometry_s> m_odometry_pub{ORB_ID(vehicle_odometry)};
    uORB::Publication<estimator_status_s> m_status_pub{ORB_ID(estimator_status)};
    uORB::Publication<estimator_status_flags_s> m_flags_pub{ORB_ID(estimator_status_flags)};
    uORB::Publication<estimator_sensor_bias_s> m_bias_pub{ORB_ID(estimator_sensor_bias)};

    state_type m_state{};
    covariance_type m_covariance{};
    types_type::parameter_type m_parameters{};
    imu_type m_previous_imu{};
    vector3_type m_accel_sum{}, m_gyro_sum{}, m_mag_sum{}, m_mag_n{};
    MapProjection m_projection{};
    vehicle_imu_s m_last_imu{};
    sensor_gps_s m_gps{};
    vehicle_magnetometer_s m_mag{};
    vehicle_air_data_s m_baro{};
    vehicle_land_detected_s m_land{};
    vehicle_status_s m_vehicle_status{};
    uint64_t m_time{}, m_origin_time{}, m_last_status{}, m_last_output{};
    uint64_t m_gps_seen{}, m_mag_seen{}, m_baro_seen{}, m_mag_init_seen{};
    uint64_t m_gps_fused{}, m_mag_fused{}, m_baro_fused{}, m_max_fusion_age{};
    uint32_t m_init_samples{}, m_init_mag_samples{}, m_predictions{}, m_gps_updates{}, m_mag_updates{},
        m_baro_updates{};
    uint32_t m_rejections{}, m_errors{}, m_gaps{};
    double m_origin_alt{}, m_origin_ellipsoid{}, m_baro_origin{};
    double m_pos_ratio{}, m_vel_ratio{}, m_height_ratio{}, m_mag_ratio{};
    bool m_initialized{}, m_fault{};
    // PX4 shell status runs on another thread; serialize diagnostic reads.
    std::mutex m_state_mutex;
    perf_counter_t m_cycle_perf{perf_alloc(PC_ELAPSED, "formal_eskf: cycle")};
}; /* end class FormalEskf */
