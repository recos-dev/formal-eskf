cmake_minimum_required(VERSION 3.21)

project(formal_eskf VERSION 0.1.0 LANGUAGES CXX)

add_library(formal_eskf_scalar_math INTERFACE)
add_library(formal_eskf::scalar_math ALIAS formal_eskf_scalar_math)

target_include_directories(
    formal_eskf_scalar_math
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_scalar_math INTERFACE cxx_std_20)

add_library(formal_eskf_linalg INTERFACE)
add_library(formal_eskf::linalg ALIAS formal_eskf_linalg)

target_include_directories(
    formal_eskf_linalg
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_linalg INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_linalg INTERFACE formal_eskf::scalar_math)

add_library(formal_eskf_so3 INTERFACE)
add_library(formal_eskf::so3 ALIAS formal_eskf_so3)

target_include_directories(
    formal_eskf_so3
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_so3 INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_so3 INTERFACE formal_eskf::linalg)

add_library(formal_eskf_core INTERFACE)
add_library(formal_eskf::core ALIAS formal_eskf_core)

target_include_directories(
    formal_eskf_core
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_core INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_core INTERFACE formal_eskf::so3)

option(ESKF_QUAT_APPROX "Use normalized Euler and first-order attitude error prediction" OFF)
option(ESKF_RESET_APPROX "Use first-order attitude-error covariance reset instead of the SO(3) right Jacobian" OFF)
target_compile_definitions(
    formal_eskf_core
    INTERFACE
        ESKF_QUAT_APPROX=$<BOOL:${ESKF_QUAT_APPROX}>
        ESKF_RESET_APPROX=$<BOOL:${ESKF_RESET_APPROX}>
)

option(FORMAL_ESKF_BUILD_EIGEN_BACKEND "Build the Eigen linear-algebra backend" ON)

if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
    find_package(Eigen3 3.3 CONFIG REQUIRED)

    add_library(formal_eskf_linalg_eigen INTERFACE)
    add_library(formal_eskf::linalg_eigen ALIAS formal_eskf_linalg_eigen)
    target_link_libraries(
        formal_eskf_linalg_eigen
        INTERFACE
            formal_eskf::linalg
            Eigen3::Eigen
    )
endif()

option(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND "Build the adapter for host-provided PX4 matrix" OFF)
set(FORMAL_ESKF_PX4_MATRIX_TARGET "" CACHE STRING "Optional existing target providing PX4 matrix's build environment")
set(FORMAL_ESKF_PX4_INCLUDE_DIRS "" CACHE STRING "Optional PX4 matrix, platform and generated-header include directories")

if(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND)
    add_library(formal_eskf_linalg_px4_matrix INTERFACE)
    add_library(formal_eskf::linalg_px4_matrix ALIAS formal_eskf_linalg_px4_matrix)
    target_link_libraries(formal_eskf_linalg_px4_matrix INTERFACE formal_eskf::linalg)

    # PX4 owns the dependency. No submodule, download, or PX4 root build here.
    # An embedding PX4 build may already supply its includes/definitions.
    if(FORMAL_ESKF_PX4_MATRIX_TARGET)
        if(NOT TARGET ${FORMAL_ESKF_PX4_MATRIX_TARGET})
            message(FATAL_ERROR "FORMAL_ESKF_PX4_MATRIX_TARGET must name an existing CMake target")
        endif()
        target_link_libraries(formal_eskf_linalg_px4_matrix INTERFACE ${FORMAL_ESKF_PX4_MATRIX_TARGET})
    endif()
    if(FORMAL_ESKF_PX4_INCLUDE_DIRS)
        target_include_directories(formal_eskf_linalg_px4_matrix SYSTEM INTERFACE ${FORMAL_ESKF_PX4_INCLUDE_DIRS})
    endif()
endif()

option(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND "Build the adapter for host-provided CMSIS-DSP" OFF)
set(FORMAL_ESKF_CMSIS_DSP_TARGET "CMSISDSP::CMSISDSP" CACHE STRING "Existing or installed target providing CMSIS-DSP")

if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
    # The embedding build owns CMSIS-DSP and its CPU/FPU/ABI/SIMD configuration.
    # Accept an existing target or an installed package; never fetch sources.
    if(FORMAL_ESKF_CMSIS_DSP_TARGET STREQUAL "CMSISDSP::CMSISDSP" AND
       NOT TARGET CMSISDSP::CMSISDSP)
        find_package(CMSISDSP CONFIG REQUIRED)
    endif()
    if(NOT TARGET "${FORMAL_ESKF_CMSIS_DSP_TARGET}")
        message(FATAL_ERROR "FORMAL_ESKF_CMSIS_DSP_TARGET must name an existing CMake target")
    endif()
    add_library(formal_eskf_linalg_cmsis_dsp INTERFACE)
    add_library(formal_eskf::linalg_cmsis_dsp ALIAS formal_eskf_linalg_cmsis_dsp)
    target_link_libraries(
        formal_eskf_linalg_cmsis_dsp
        INTERFACE
            formal_eskf::linalg
            ${FORMAL_ESKF_CMSIS_DSP_TARGET}
    )
endif()

option(FORMAL_ESKF_BUILD_EXAMPLES "Build host-side replay examples" OFF)
set(FORMAL_ESKF_REPLAY_BACKEND "eigen" CACHE STRING "Replay linear-algebra backend (eigen or px4_matrix)")
set_property(CACHE FORMAL_ESKF_REPLAY_BACKEND PROPERTY STRINGS eigen px4_matrix)

if(FORMAL_ESKF_BUILD_EXAMPLES)
    if(NOT FORMAL_ESKF_REPLAY_BACKEND MATCHES "^(eigen|px4_matrix)$")
        message(FATAL_ERROR "FORMAL_ESKF_REPLAY_BACKEND must be eigen or px4_matrix")
    endif()
    if(NOT TARGET formal_eskf::linalg_${FORMAL_ESKF_REPLAY_BACKEND})
        message(FATAL_ERROR "Enable the ${FORMAL_ESKF_REPLAY_BACKEND} backend to build the replay example")
    endif()
    add_executable(replay_px4 examples/replay_px4.cpp)
    target_link_libraries(replay_px4 PRIVATE formal_eskf::core formal_eskf::linalg_${FORMAL_ESKF_REPLAY_BACKEND})
    if(FORMAL_ESKF_REPLAY_BACKEND STREQUAL "px4_matrix")
        target_compile_definitions(replay_px4 PRIVATE FORMAL_ESKF_REPLAY_PX4_MATRIX)
    endif()
    set_target_properties(replay_px4 PROPERTIES CXX_EXTENSIONS OFF)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(replay_px4 PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
    include(CTest)

    if(BUILD_TESTING)
        function(formal_eskf_configure_test target_name)
            set_target_properties(${target_name} PROPERTIES CXX_EXTENSIONS OFF)
            if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
                target_compile_options(
                    ${target_name}
                    PRIVATE
                        -Wall
                        -Wextra
                        -Wpedantic
                        -Wconversion
                        -Wsign-conversion
                )
            endif()
        endfunction()

        add_executable(test_scalar_standard tests/test_scalar_standard.cpp)
        target_link_libraries(test_scalar_standard PRIVATE formal_eskf::scalar_math)
        formal_eskf_configure_test(test_scalar_standard)
        add_test(NAME formal_eskf.scalar.standard COMMAND test_scalar_standard)

        if(FORMAL_ESKF_BUILD_EIGEN_BACKEND OR FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND OR
           FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
            # Independent eigenvalue oracle in test_eskf_covariance.cpp only.
            # PX4/CMSIS-DSP production targets never link Eigen.
            find_package(Eigen3 3.3 CONFIG REQUIRED)
        endif()

        function(formal_eskf_add_backend_test target_name source_name test_name library)
            if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
                add_executable(${target_name} tests/${source_name}.cpp)
                target_link_libraries(${target_name} PRIVATE ${library} formal_eskf::linalg_eigen)
                target_compile_definitions(${target_name} PRIVATE EIGEN_RUNTIME_NO_MALLOC)
                formal_eskf_configure_test(${target_name})
                add_test(NAME ${test_name} COMMAND ${target_name})
            endif()

            if(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND)
                set(px4_target ${target_name}_px4_matrix)
                add_executable(${px4_target} tests/${source_name}.cpp)
                target_link_libraries(${px4_target} PRIVATE ${library} formal_eskf::linalg_px4_matrix)
                target_compile_definitions(${px4_target} PRIVATE FORMAL_ESKF_TEST_PX4_MATRIX)
                if(source_name STREQUAL "test_eskf_covariance")
                    target_link_libraries(${px4_target} PRIVATE Eigen3::Eigen)
                endif()
                formal_eskf_configure_test(${px4_target})
                string(REGEX REPLACE "\\.eigen$" "" px4_test_name "${test_name}")
                add_test(NAME ${px4_test_name}.px4_matrix COMMAND ${px4_target})
            endif()

            if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
                set(cmsis_target ${target_name}_cmsis_dsp)
                add_executable(${cmsis_target} tests/${source_name}.cpp)
                target_link_libraries(${cmsis_target} PRIVATE ${library} formal_eskf::linalg_cmsis_dsp)
                target_compile_definitions(${cmsis_target} PRIVATE FORMAL_ESKF_TEST_CMSIS_DSP)
                if(source_name STREQUAL "test_eskf_covariance")
                    target_link_libraries(${cmsis_target} PRIVATE Eigen3::Eigen)
                endif()
                formal_eskf_configure_test(${cmsis_target})
                string(REGEX REPLACE "\\.eigen$" "" cmsis_test_name "${test_name}")
                add_test(NAME ${cmsis_test_name}.cmsis_dsp COMMAND ${cmsis_target})
            endif()
        endfunction()

        if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
            add_executable(test_cmsis_dsp_backend tests/test_cmsis_dsp_backend.cpp)
            target_link_libraries(test_cmsis_dsp_backend PRIVATE formal_eskf::linalg_cmsis_dsp)
            formal_eskf_configure_test(test_cmsis_dsp_backend)
            add_test(NAME formal_eskf.linalg.cmsis_dsp.storage COMMAND test_cmsis_dsp_backend)
        endif()

        formal_eskf_add_backend_test(test_linalg_eigen test_linalg_eigen formal_eskf.linalg.eigen formal_eskf::linalg)
        formal_eskf_add_backend_test(test_linalg_spd test_linalg_spd formal_eskf.linalg.spd formal_eskf::linalg)
        formal_eskf_add_backend_test(test_so3_eigen test_so3_eigen formal_eskf.so3.eigen formal_eskf::so3)
        formal_eskf_add_backend_test(test_so3_jacobian test_so3_jacobian formal_eskf.so3.jacobian formal_eskf::so3)
        formal_eskf_add_backend_test(test_eskf_types test_eskf_types formal_eskf.types.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_prediction test_eskf_prediction formal_eskf.prediction.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_covariance test_eskf_covariance formal_eskf.covariance.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_injection test_eskf_injection formal_eskf.injection.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_correction test_eskf_correction formal_eskf.correction.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_position test_eskf_position formal_eskf.position.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_velocity test_eskf_velocity formal_eskf.velocity.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_magnetometer test_eskf_magnetometer formal_eskf.magnetometer.eigen formal_eskf::core)
        formal_eskf_add_backend_test(test_eskf_accelerometer test_eskf_accelerometer formal_eskf.accelerometer.eigen formal_eskf::core)
    endif()
endif()
