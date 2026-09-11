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
        endif()
    endif()
endif()
