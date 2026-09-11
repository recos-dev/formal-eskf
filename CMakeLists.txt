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

        if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
            add_executable(test_linalg_eigen tests/test_linalg_eigen.cpp)
            target_link_libraries(test_linalg_eigen PRIVATE formal_eskf::linalg_eigen)
            target_compile_definitions(test_linalg_eigen PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_linalg_eigen)
            add_test(NAME formal_eskf.linalg.eigen COMMAND test_linalg_eigen)

            add_executable(test_linalg_spd tests/test_linalg_spd.cpp)
            target_link_libraries(test_linalg_spd PRIVATE formal_eskf::linalg_eigen)
            target_compile_definitions(test_linalg_spd PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_linalg_spd)
            add_test(NAME formal_eskf.linalg.spd COMMAND test_linalg_spd)

            add_executable(test_so3_eigen tests/test_so3_eigen.cpp)
            target_link_libraries(
                test_so3_eigen
                PRIVATE
                    formal_eskf::so3
                    formal_eskf::linalg_eigen
            )
            target_compile_definitions(test_so3_eigen PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_so3_eigen)
            add_test(NAME formal_eskf.so3.eigen COMMAND test_so3_eigen)

            add_executable(test_so3_jacobian tests/test_so3_jacobian.cpp)
            target_link_libraries(test_so3_jacobian PRIVATE formal_eskf::so3 formal_eskf::linalg_eigen)
            target_compile_definitions(test_so3_jacobian PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_so3_jacobian)
            add_test(NAME formal_eskf.so3.jacobian COMMAND test_so3_jacobian)

            add_executable(test_eskf_types tests/test_eskf_types.cpp)
            target_link_libraries(
                test_eskf_types
                PRIVATE
                    formal_eskf::core
                    formal_eskf::linalg_eigen
            )
            target_compile_definitions(test_eskf_types PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_types)
            add_test(NAME formal_eskf.types.eigen COMMAND test_eskf_types)

            add_executable(test_eskf_prediction tests/test_eskf_prediction.cpp)
            target_link_libraries(test_eskf_prediction PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_prediction PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_prediction)
            add_test(NAME formal_eskf.prediction.eigen COMMAND test_eskf_prediction)

            add_executable(test_eskf_covariance tests/test_eskf_covariance.cpp)
            target_link_libraries(test_eskf_covariance PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_covariance PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_covariance)
            add_test(NAME formal_eskf.covariance.eigen COMMAND test_eskf_covariance)

            add_executable(test_eskf_injection tests/test_eskf_injection.cpp)
            target_link_libraries(test_eskf_injection PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_injection PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_injection)
            add_test(NAME formal_eskf.injection.eigen COMMAND test_eskf_injection)

            add_executable(test_eskf_correction tests/test_eskf_correction.cpp)
            target_link_libraries(test_eskf_correction PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_correction PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_correction)
            add_test(NAME formal_eskf.correction.eigen COMMAND test_eskf_correction)

            add_executable(test_eskf_position tests/test_eskf_position.cpp)
            target_link_libraries(test_eskf_position PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_position PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_position)
            add_test(NAME formal_eskf.position.eigen COMMAND test_eskf_position)

            add_executable(test_eskf_velocity tests/test_eskf_velocity.cpp)
            target_link_libraries(test_eskf_velocity PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_velocity PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_velocity)
            add_test(NAME formal_eskf.velocity.eigen COMMAND test_eskf_velocity)

            add_executable(test_eskf_magnetometer tests/test_eskf_magnetometer.cpp)
            target_link_libraries(test_eskf_magnetometer PRIVATE formal_eskf::core formal_eskf::linalg_eigen)
            target_compile_definitions(test_eskf_magnetometer PRIVATE EIGEN_RUNTIME_NO_MALLOC)
            formal_eskf_configure_test(test_eskf_magnetometer)
            add_test(NAME formal_eskf.magnetometer.eigen COMMAND test_eskf_magnetometer)
        endif()
    endif()
endif()
