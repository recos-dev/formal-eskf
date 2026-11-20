function(formal_eskf_configure_test target_name)
    set_target_properties(${target_name} PROPERTIES
        CXX_EXTENSIONS OFF
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/support)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(
            ${target_name}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -UNDEBUG
                -fno-fast-math
                -ffp-contract=off
        )
    endif()
endfunction()

add_executable(test_scalar_standard math/test_scalar.cpp)
target_link_libraries(test_scalar_standard PRIVATE formal_eskf::scalar_math)
formal_eskf_configure_test(test_scalar_standard)
add_test(NAME formal_eskf.scalar.standard COMMAND test_scalar_standard)
set_tests_properties(formal_eskf.scalar.standard PROPERTIES LABELS "plan-source:tests/math/test_scalar.cpp")

add_executable(test_case_runner tools/test_case_runner.cpp)
target_link_libraries(test_case_runner PRIVATE formal_eskf::scalar_math)
formal_eskf_configure_test(test_case_runner)
add_test(NAME formal_eskf.test_cases.TC-TOOL-test-cases-selection.host COMMAND test_case_runner)

add_executable(test_ring_buffer runtime/test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE formal_eskf::runtime)
formal_eskf_configure_test(test_ring_buffer)
add_test(NAME formal_eskf.runtime.ring_buffer COMMAND test_ring_buffer)
set_tests_properties(formal_eskf.runtime.ring_buffer PROPERTIES LABELS "plan-source:tests/runtime/test_ring_buffer.cpp")

if(TARGET replay_ins)
    foreach(CASE IN ITEMS known missing clipped disabled gap limits zero_limits invalid_limits)
        add_test(
            NAME formal_eskf.replay_bias.${CASE}
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:replay_ins>
                -DOUTPUT_DIR=${PROJECT_BINARY_DIR}
                -DCASE=${CASE}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/test_replay_bias.cmake
        )
    endforeach()
    foreach(CASE IN ITEMS fault height activation reset_quality reset_gap reset_metadata_gap horizontal speed_accuracy negative clear missing gap metadata_gap quality batch outage)
        add_test(
            NAME formal_eskf.replay_vertical_acceleration.${CASE}
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:replay_ins>
                -DOUTPUT_DIR=${PROJECT_BINARY_DIR}
                -DCASE=${CASE}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/test_replay_vertical_acceleration.cmake
        )
    endforeach()
    foreach(CASE IN ITEMS timeout disturbed aided data_gap_unaided data_gap_aided
                          disturbed_aiding_recovery rejected_aiding_recovery
                          source_ground source_air source_unaided
                          source_unknown calibration source_disturbed reference_unaided reference_aided context_invalid
                          model_aided model_rejected_position model_unavailable
                          model_health_only model_health_disturbed
                          field_required field_fallback field_disabled field_force_only model_required)
        add_test(
            NAME formal_eskf.replay_magnetometer.reset.${CASE}
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:replay_ins>
                -DOUTPUT_DIR=${PROJECT_BINARY_DIR}
                -DCASE=${CASE}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/test_replay_magnetometer_reset.cmake
        )
    endforeach()
    foreach(GROUP IN ITEMS velocity horizontal_position vertical_position)
        add_test(
            NAME formal_eskf.replay_gnss.failure.${GROUP}
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:replay_ins>
                -DOUTPUT_DIR=${PROJECT_BINARY_DIR}
                -DGROUP=${GROUP}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/test_replay_gnss_failure.cmake
        )
    endforeach()
    foreach(CASE IN ITEMS startup both_timeout position_timeout reset_failure position_reset_failure
            height_reset_failure height_reentry height_timeout height_overdue_success full_outage all_timeout
            vertical_velocity_reset)
        add_test(
            NAME formal_eskf.replay_gnss.reset.${CASE}
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:replay_ins>
                -DOUTPUT_DIR=${PROJECT_BINARY_DIR}
                -DCASE=${CASE}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/test_replay_gnss_reset.cmake
        )
    endforeach()
endif()

if(FORMAL_ESKF_BUILD_EIGEN_BACKEND OR FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND OR
   FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
    # Independent eigenvalue oracle in core/test_covariance.cpp only.
    # PX4/CMSIS-DSP production targets never link Eigen.
    find_package(Eigen3 3.3 CONFIG REQUIRED)
endif()

function(formal_eskf_add_backend_test target_name source_name test_name library)
    if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
        add_executable(${target_name} ${source_name}.cpp)
        target_link_libraries(${target_name} PRIVATE ${library} formal_eskf::linalg_eigen)
        target_compile_definitions(${target_name} PRIVATE EIGEN_RUNTIME_NO_MALLOC)
        formal_eskf_configure_test(${target_name})
        add_test(NAME ${test_name} COMMAND ${target_name})
        set_tests_properties(${test_name} PROPERTIES LABELS "plan-source:tests/${source_name}.cpp")
    endif()

    if(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND)
        set(px4_target ${target_name}_px4_matrix)
        add_executable(${px4_target} ${source_name}.cpp)
        target_link_libraries(${px4_target} PRIVATE ${library} formal_eskf::linalg_px4_matrix)
        target_compile_definitions(${px4_target} PRIVATE FORMAL_ESKF_TEST_PX4_MATRIX)
        if(target_name STREQUAL "test_eskf_covariance")
            target_link_libraries(${px4_target} PRIVATE Eigen3::Eigen)
        endif()
        formal_eskf_configure_test(${px4_target})
        string(REGEX REPLACE "\\.eigen$" "" px4_test_name "${test_name}")
        add_test(NAME ${px4_test_name}.px4_matrix COMMAND ${px4_target})
        set_tests_properties(${px4_test_name}.px4_matrix PROPERTIES LABELS "plan-source:tests/${source_name}.cpp")
    endif()

    if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
        set(cmsis_target ${target_name}_cmsis_dsp)
        add_executable(${cmsis_target} ${source_name}.cpp)
        target_link_libraries(${cmsis_target} PRIVATE ${library} formal_eskf::linalg_cmsis_dsp)
        target_compile_definitions(${cmsis_target} PRIVATE FORMAL_ESKF_TEST_CMSIS_DSP)
        if(target_name STREQUAL "test_eskf_covariance")
            target_link_libraries(${cmsis_target} PRIVATE Eigen3::Eigen)
        endif()
        formal_eskf_configure_test(${cmsis_target})
        string(REGEX REPLACE "\\.eigen$" "" cmsis_test_name "${test_name}")
        add_test(NAME ${cmsis_test_name}.cmsis_dsp COMMAND ${cmsis_target})
        set_tests_properties(${cmsis_test_name}.cmsis_dsp PROPERTIES LABELS "plan-source:tests/${source_name}.cpp")
    endif()
endfunction()

if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
    add_executable(test_cmsis_dsp_backend math/test_cmsis_dsp_backend.cpp)
    target_link_libraries(test_cmsis_dsp_backend PRIVATE formal_eskf::linalg_cmsis_dsp)
    formal_eskf_configure_test(test_cmsis_dsp_backend)
    add_test(NAME formal_eskf.linalg.cmsis_dsp.storage COMMAND test_cmsis_dsp_backend)
    set_tests_properties(formal_eskf.linalg.cmsis_dsp.storage PROPERTIES LABELS "plan-source:tests/math/test_cmsis_dsp_backend.cpp")
endif()

formal_eskf_add_backend_test(test_linalg_eigen math/test_linalg formal_eskf.linalg.eigen formal_eskf::linalg)
formal_eskf_add_backend_test(test_linalg_spd math/test_spd_solve formal_eskf.linalg.spd formal_eskf::linalg)
formal_eskf_add_backend_test(test_so3_eigen math/test_so3 formal_eskf.so3.eigen formal_eskf::so3)
formal_eskf_add_backend_test(test_so3_jacobian math/test_so3_jacobian formal_eskf.so3.jacobian formal_eskf::so3)
formal_eskf_add_backend_test(test_eskf_types core/test_types formal_eskf.types.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_sequence test_eskf_sequence formal_eskf.sequence.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_prediction core/test_prediction formal_eskf.prediction.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_covariance core/test_covariance formal_eskf.covariance.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_injection core/test_injection_reset formal_eskf.injection.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_correction core/test_correction formal_eskf.correction.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_codegen test_codegen formal_eskf.codegen.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_eskf_innovation core/test_innovation formal_eskf.innovation.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_gnss_quality test_gnss_quality formal_eskf.gnss_quality.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_gnss_fusion test_gnss_fusion formal_eskf.gnss_fusion.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_navigation_measurement test_navigation_measurement formal_eskf.navigation_measurement.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_navigation_reset test_navigation_reset formal_eskf.navigation_reset.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_bias_process_noise test_bias_process_noise formal_eskf.bias_process_noise.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_imu_process_noise runtime/imu/test_process_noise formal_eskf.imu_process_noise.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_bias_limits runtime/imu/test_bias_limits formal_eskf.bias_limits.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_vertical_acceleration_health test_vertical_acceleration_health formal_eskf.vertical_acceleration_health.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_accelerometer_magnitude runtime/gravity/test_magnitude formal_eskf.accelerometer_magnitude.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_accelerometer_conditions runtime/gravity/test_conditions formal_eskf.accelerometer_conditions.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_accelerometer_fusion test_accelerometer_fusion formal_eskf.accelerometer_fusion.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_replay_conditions test_replay_conditions formal_eskf.replay_conditions.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_imu_bias_inhibition runtime/imu/test_bias_inhibition formal_eskf.imu_bias_inhibition.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_replay_gnss test_replay_gnss formal_eskf.replay_gnss.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_accelerometer_sample runtime/gravity/test_sampling formal_eskf.accelerometer_sample.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_measurement_fusion test_measurement_fusion formal_eskf.measurement_fusion.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_magnetic_model test_px4_magnetic_model formal_eskf.px4_magnetic_model.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_height_aiding test_px4_height_aiding formal_eskf.px4_height_aiding.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_imu_prediction test_px4_imu_prediction formal_eskf.px4_imu_prediction.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_imu_history test_px4_imu_history formal_eskf.px4_imu_history.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_output_predictor test_px4_output_predictor formal_eskf.px4_output_predictor.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_px4_observation_history test_px4_observation_history formal_eskf.px4_observation_history.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_magnetometer_fusion test_magnetometer_fusion formal_eskf.magnetometer_fusion.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_magnetometer_lifecycle test_magnetometer_lifecycle formal_eskf.magnetometer_lifecycle.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_magnetic_heading test_magnetic_heading formal_eskf.magnetic_heading.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_magnetic_reference test_magnetic_reference formal_eskf.magnetic_reference.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_magnetometer_reset test_magnetometer_reset formal_eskf.magnetometer_reset.eigen formal_eskf::runtime)
formal_eskf_add_backend_test(test_eskf_position test_eskf_position formal_eskf.position.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_velocity test_eskf_velocity formal_eskf.velocity.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_magnetometer test_eskf_magnetometer formal_eskf.magnetometer.eigen formal_eskf::core)
formal_eskf_add_backend_test(test_eskf_accelerometer test_eskf_accelerometer formal_eskf.accelerometer.eigen formal_eskf::core)

formal_eskf_add_backend_test(test_contracts common/test_contracts formal_eskf.common.contracts formal_eskf::runtime)

formal_eskf_add_backend_test(test_tilt_alignment runtime/test_tilt_alignment formal_eskf.tilt_alignment.eigen formal_eskf::runtime)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
    separate_arguments(TEST_COMPILER_ARGS NATIVE_COMMAND "${CMAKE_CXX_COMPILER_ARG1} ${CMAKE_CXX_FLAGS}")
    list(TRANSFORM TEST_COMPILER_ARGS PREPEND "--compiler-arg=")
    add_test(NAME formal_eskf.common.compile_contracts
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/common/test_compile_contracts.py
            --compiler ${CMAKE_CXX_COMPILER} ${TEST_COMPILER_ARGS}
            --include-dir $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>)
    set_tests_properties(formal_eskf.common.compile_contracts PROPERTIES LABELS "plan-source:tests/common/test_compile_contracts.py")
endif()

add_test(NAME formal_eskf.tools.test_plan
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_test_plan.py)
