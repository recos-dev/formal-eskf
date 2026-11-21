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

add_library(formal_eskf_geodesy INTERFACE)
add_library(formal_eskf::geodesy ALIAS formal_eskf_geodesy)
target_link_libraries(formal_eskf_geodesy INTERFACE formal_eskf::linalg)

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

option(ESKF_CODEGEN "Use generated INS matrix arithmetic" ON)
option(ESKF_QUAT_APPROX "Use normalized Euler and first-order attitude error prediction" OFF)
option(ESKF_RESET_APPROX "Use first-order attitude-error covariance reset instead of the SO(3) right Jacobian" OFF)
option(ESKF_MAG_TILT "Restrict magnetic attitude correction to rotation about navigation Down" ON)
target_compile_definitions(
    formal_eskf_core
    INTERFACE
        ESKF_CODEGEN=$<BOOL:${ESKF_CODEGEN}>
        ESKF_QUAT_APPROX=$<BOOL:${ESKF_QUAT_APPROX}>
        ESKF_RESET_APPROX=$<BOOL:${ESKF_RESET_APPROX}>
        ESKF_MAG_TILT=$<BOOL:${ESKF_MAG_TILT}>
)

add_library(formal_eskf_runtime INTERFACE)
add_library(formal_eskf::runtime ALIAS formal_eskf_runtime)
target_link_libraries(formal_eskf_runtime INTERFACE formal_eskf::core)

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
set(FORMAL_ESKF_PX4_WMM_TARGET "" CACHE STRING "Optional host target providing PX4 magnetic-model functions and headers")
set(FORMAL_ESKF_REPLAY_BACKEND "eigen" CACHE STRING "Replay linear-algebra backend (eigen or px4_matrix)")
set_property(CACHE FORMAL_ESKF_REPLAY_BACKEND PROPERTY STRINGS eigen px4_matrix)

if(FORMAL_ESKF_BUILD_EXAMPLES)
    if(NOT FORMAL_ESKF_REPLAY_BACKEND MATCHES "^(eigen|px4_matrix)$")
        message(FATAL_ERROR "FORMAL_ESKF_REPLAY_BACKEND must be eigen or px4_matrix")
    endif()
    if(NOT TARGET formal_eskf::linalg_${FORMAL_ESKF_REPLAY_BACKEND})
        message(FATAL_ERROR "Enable the ${FORMAL_ESKF_REPLAY_BACKEND} backend to build the replay example")
    endif()
    add_executable(replay_ins examples/replay_ins.cpp)
    target_link_libraries(replay_ins PRIVATE formal_eskf::runtime formal_eskf::geodesy formal_eskf::linalg_${FORMAL_ESKF_REPLAY_BACKEND})
    if(FORMAL_ESKF_REPLAY_BACKEND STREQUAL "px4_matrix")
        target_compile_definitions(replay_ins PRIVATE FORMAL_ESKF_REPLAY_PX4_MATRIX)
    endif()
    set_target_properties(replay_ins PROPERTIES CXX_EXTENSIONS OFF)
    if(FORMAL_ESKF_PX4_WMM_TARGET)
        if(NOT TARGET ${FORMAL_ESKF_PX4_WMM_TARGET})
            message(FATAL_ERROR "FORMAL_ESKF_PX4_WMM_TARGET must name an existing host target")
        endif()
        add_executable(px4_magnetic_model examples/px4_magnetic_model.cpp)
        target_link_libraries(px4_magnetic_model PRIVATE ${FORMAL_ESKF_PX4_WMM_TARGET})
        target_compile_features(px4_magnetic_model PRIVATE cxx_std_20)
        set_target_properties(px4_magnetic_model PROPERTIES CXX_EXTENSIONS OFF)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(px4_magnetic_model PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
        endif()
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(replay_ins PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
    include(CTest)

    if(BUILD_TESTING)
        add_subdirectory(tests)
    endif()
endif()
