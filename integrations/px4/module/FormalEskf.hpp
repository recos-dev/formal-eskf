// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
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
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/imu.hpp>
#include <formal_eskf/runtime/accelerometer.hpp>
#include <formal_eskf/runtime/tilt_alignment.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <formal_eskf/runtime/magnetometer_lifecycle.hpp>

#include "DeploymentProfile.hpp"
#include "HeightAiding.hpp"
#include "ImuPrediction.hpp"
#include "MagneticModel.hpp"

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
    friend class FormalEskfModuleTest;

    using backend_type = formal_eskf::linalg::Px4MatrixBackend<double>;
    using types_type = formal_eskf::EskfTypes<backend_type, formal_eskf::configuration::Ins>;
    using state_type = types_type::nominal_state_type;
    using covariance_type = types_type::error_covariance_type;
    using vector3_type = formal_eskf::linalg::Vector<backend_type, 3>;
    using matrix3_type = formal_eskf::linalg::Matrix<backend_type, 3, 3>;
    using imu_type = types_type::imu_sample_type;
    using status_type = formal_eskf::Status;
    using height_type = formal_eskf::px4::HeightAiding<backend_type>;
    using bias_learning_type = formal_eskf::runtime::ImuBiasInhibition<backend_type>;
    using vertical_health_type = formal_eskf::runtime::VerticalAccelerationHealth<backend_type>;
    using gravity_sampler_type =
        formal_eskf::runtime::AccelerometerSample<backend_type,
                                                  formal_eskf::px4::profile::gravity::observation_interval_us>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<backend_type>;
    using magnetic_lifecycle_type = formal_eskf::runtime::MagnetometerFusion;
    using magnetic_model_type = formal_eskf::px4::MagneticModel<backend_type>;

    struct Initialization
    {
        vector3_type accelerometer_sum{};
        vector3_type gyroscope_sum{};
        vector3_type magnetic_sum_gauss{};
        std::uint32_t imu_samples{};
        std::uint32_t magnetic_samples{};
        std::uint64_t magnetic_time_us{};
        magnetic_lifecycle_type::Source magnetic_source{};

        // Motion resets accumulated observations, not sensor identity/watermarks.
        void reset_samples();
    };

    struct ImuSource
    {
        std::uint32_t accelerometer_id{};
        std::uint32_t gyroscope_id{};
        std::uint8_t accelerometer_calibration{};
        std::uint8_t gyroscope_calibration{};

        bool matches(ImuSource const & other) const;
    };

    // Integrated body-FRD input, independent of PX4 message layout. Keep both
    // measured periods: gravity sampling consumes microseconds, not rounded dt.
    struct ImuInput
    {
        vector3_type delta_angle_b{};    // rad
        vector3_type delta_velocity_b{}; // m/s
        std::uint64_t time_us{};
        std::uint32_t angular_period_us{};
        std::uint32_t acceleration_period_us{};
        std::uint8_t accelerometer_clipping{};
        std::uint8_t gyroscope_clipping{};
        ImuSource source{};

        double angular_interval_s() const;
        double acceleration_interval_s() const;
        imu_type mean_rates() const; // Requires admitted, positive periods.
    };

    struct ImuHistory
    {
        imu_type measurement{};
        std::uint64_t time_us{};
        ImuSource source{};

        void record(ImuInput const & sample, imu_type const & imu);
    };

    struct NavigationOrigin
    {
        MapProjection projection{};
        std::uint64_t time_us{};
        double altitude_msl_m{};
        double altitude_ellipsoid_m{};
    };

    struct Gravity
    {
        gravity_sampler_type sampler{};
        bool active{};
        std::uint32_t samples{};
        std::uint32_t corrections{};
        std::uint64_t interval_us{};

        // Discard partial observations/LPF history, retaining session counters.
        void reset_samples();
    };

    struct Magnetic
    {
        formal_eskf::runtime::MagneticHeading<backend_type> heading{};
        formal_eskf::runtime::MagnetometerFilter<backend_type> filter{};
        magnetic_lifecycle_type lifecycle{};
        formal_eskf::runtime::MagnetometerFusionResult<backend_type> observation{};
        magnetic_lifecycle_type::Source source{};
        std::uint64_t sample_time_us{}; // Newest consumed arrival, including stale/invalid data.
        bool horizontal_aiding{};
        std::uint32_t corrections{};
        double test_ratio{};
    };

    struct MagneticSample
    {
        vector3_type field_b{}; // Calibrated body FRD, microtesla.
        magnetic_lifecycle_type::Source source{};
        std::uint64_t time_us{}; // Zero denotes no eligible new arrival.
        bool fresh{};
        bool usable{};
    };

    void Run() override;
    static ImuInput decode_imu(vehicle_imu_s const & sample);
    void process_imu(ImuInput const & sample);
    void predict_and_fuse(ImuInput const & sample, imu_type const & imu);
    void initialize(ImuInput const & sample, imu_type const & imu);
    bool collect_initialization_samples(imu_type const & imu);
    bool initialization_ready() const;
    bool initialize_attitude(double & yaw);
    void initialize_navigation();
    static types_type::parameter_type estimator_parameters();
    static bias_learning_type::Parameters bias_learning_parameters();
    bool load_startup_parameters();
    bool accept_imu(ImuInput const & sample);
    void reset_imu_observations();
    void fuse_gps();
    void fuse_mag();
    void initialize_magnetic_aiding(magnetic_lifecycle_type::Source const & source, std::uint64_t sample_time_us);
    magnetic_lifecycle_type::Conditions magnetic_conditions() const;
    MagneticSample consume_magnetic_sample();
    status_type prepare_magnetic_observation(MagneticSample const & sample,
                                             magnetic_lifecycle_type::Conditions const & conditions,
                                             magnetic_lifecycle_type::Sample & event);
    bool fuse_magnetic_observation(MagneticSample const & sample);
    status_type reset_magnetic_heading(std::uint64_t sample_time_us);
    void fuse_height();
    void fuse_gravity(ImuInput const & sample, imu_type const & imu);
    void record_vertical_velocity_reset(double delta);
    bool update_imu_bias(ImuInput const & sample, imu_type const & imu);
    bool update_imu_health(std::uint8_t accelerometer_clipping);
    void publish(imu_type const & imu);
    void publish_status(uint64_t now);
    void update_gps_quality();
    void update_magnetic_model();
    std::optional<magnetic_model_type::sample_type> lookup_magnetic_model(double latitude, double longitude) const;
    bool gps_good() const;
    bool magnetic_heading_valid() const;
    bool magnetic_heading_aiding() const;
    uint16_t gps_failure_flags() const;
    bool gps_position_valid(uint64_t max_age = 5000000) const;
    bool gps_velocity_valid(uint64_t max_age = 5000000) const;
    template <std::size_t N>
    bool record_gps_fusion(formal_eskf::runtime::GnssFusion::Group group,
                           formal_eskf::runtime::FusionResult<backend_type, N> const & result, double & ratio);
    bool check(status_type status);
    bool recent(uint64_t sample, uint64_t max_age = 1000000) const;
    static vector3_type vector3(double x, double y, double z) { return vector3_type::from_row_major({x, y, z}); }
    static double square(double x) { return x * x; }

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
    types_type::parameter_type m_parameters = estimator_parameters();
    Initialization m_initialization{};
    ImuHistory m_imu_history{};
    NavigationOrigin m_origin{};
    std::uint64_t m_time_us{}; // Last admitted IMU horizon, not wall-clock time.
    bool m_initialized{};
    bool m_fault{};

    height_type m_height_aiding{};
    formal_eskf::runtime::TiltAlignment<backend_type> m_tilt_alignment{};
    Gravity m_gravity{};
    bool m_gravity_enabled = true;
    bias_learning_type m_bias_learning{bias_learning_parameters()};
    bias_learning_type::Result m_bias_learning_result{};
    vertical_health_type m_vertical_health{};
    vertical_health_type::Innovation m_gps_velocity_innovation{};
    double m_average_imu_interval = formal_eskf::px4::profile::imu::nominal_interval_s;
    Magnetic m_magnetic{};
    bool m_heading_aligned{};
    quaternion_type m_quaternion_reset_delta{};
    std::uint8_t m_quaternion_reset_counter{};
    magnetic_model_type m_magnetic_model{};
    magnetic_model_type::Settings m_magnetic_settings{};
    uint64_t m_mag_model_origin_time{};
    uint32_t m_mag_model_unavailable{};
    MapProjection m_gps_previous_fix{};
    formal_eskf::runtime::GnssQuality<backend_type> m_gnss_quality{};
    formal_eskf::runtime::GnssQualityParameters<double> m_gnss_quality_parameters{};
    formal_eskf::runtime::GnssQualityResult<double> m_gnss_quality_result{};
    formal_eskf::runtime::GnssFusion m_gnss_fusion{};
    formal_eskf::runtime::GnssFusion::Sample m_gnss_sample{};
    // Horizontal-only profile; HeightAiding owns GNSS/barometer height arbitration.
    static constexpr formal_eskf::runtime::GnssFusion::Parameters gnss_fusion_parameters{7000000, 1000000, 5000000};
    double m_gps_previous_alt{};
    uint32_t m_gps_device_id{};
    bool m_gps_metadata_valid{};
    formal_eskf::linalg::Vector<backend_type, 2> m_position_reset_delta{};
    vector3_type m_velocity_reset_delta{};
    uint8_t m_position_reset_counter{}, m_velocity_reset_counter{}, m_vertical_velocity_reset_counter{},
        m_odometry_reset_counter{};
    double m_vertical_velocity_reset_delta{};
    uint64_t m_vertical_velocity_reset_time{};
    sensor_gps_s m_gps{};
    vehicle_magnetometer_s m_mag{};
    vehicle_air_data_s m_baro{};
    vehicle_land_detected_s m_land{};
    vehicle_status_s m_vehicle_status{};
    uint64_t m_last_status{}, m_last_output{};
    uint64_t m_gps_seen{}, m_max_fusion_age{};
    uint32_t m_predictions{}, m_gps_updates{}, m_baro_updates{};
    uint32_t m_rejections{}, m_errors{}, m_gaps{};
    double m_pos_ratio{}, m_vel_ratio{}, m_height_ratio{}, m_gps_height_ratio{};
    // PX4 shell status runs on another thread; serialize diagnostic reads.
    std::mutex m_state_mutex;
    perf_counter_t m_cycle_perf{perf_alloc(PC_ELAPSED, "formal_eskf: cycle")};
}; /* end class FormalEskf */
