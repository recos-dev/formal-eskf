cmake_minimum_required(VERSION 3.21)

project(formal_eskf VERSION 0.1.0 LANGUAGES CXX)

add_library(formal_eskf_linalg INTERFACE)
add_library(formal_eskf::linalg ALIAS formal_eskf_linalg)

target_include_directories(
    formal_eskf_linalg
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_linalg INTERFACE cxx_std_20)

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

    if(BUILD_TESTING AND FORMAL_ESKF_BUILD_EIGEN_BACKEND)
        add_executable(test_linalg_eigen tests/test_linalg_eigen.cpp)
        target_link_libraries(test_linalg_eigen PRIVATE formal_eskf::linalg_eigen)
        target_compile_definitions(test_linalg_eigen PRIVATE EIGEN_RUNTIME_NO_MALLOC)
        set_target_properties(test_linalg_eigen PROPERTIES CXX_EXTENSIONS OFF)

        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(
                test_linalg_eigen
                PRIVATE
                    -Wall
                    -Wextra
                    -Wpedantic
                    -Wconversion
                    -Wsign-conversion
            )
        endif()

        add_test(NAME formal_eskf.linalg.eigen COMMAND test_linalg_eigen)
    endif()
endif()
