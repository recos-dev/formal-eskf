# install.py copies the core into this module. No source checkout is needed.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/core/include/formal_eskf/eskf/eskf.hpp")
	message(FATAL_ERROR "Install the complete formal-eskf PX4 package, including core/include")
endif()

# Keep all translation units, including the module fixture, on the packaged profile.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/DeploymentConfig.hpp" FORMAL_ESKF_CONFIG)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/DeploymentConfig.hpp")
foreach(OPTION QUAT_APPROX RESET_APPROX MAG_TILT)
	if(NOT FORMAL_ESKF_CONFIG MATCHES "#define ESKF_CFG_${OPTION} ([01])[\r\n]")
		message(FATAL_ERROR "Missing or invalid ESKF_CFG_${OPTION} in DeploymentConfig.hpp")
	endif()
	list(APPEND FORMAL_ESKF_ALGORITHMS "ESKF_${OPTION}=${CMAKE_MATCH_1}")
endforeach()

px4_add_module(
	MODULE modules__formal_eskf
	MAIN formal_eskf
	SRCS
		FormalEskf.cpp
		FormalEskfNavigation.cpp
		FormalEskfAttitude.cpp
		FormalEskfOutput.cpp
	DEPENDS px4_work_queue geo world_magnetic_model
)
target_include_directories(modules__formal_eskf PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/core/include")
set_target_properties(modules__formal_eskf PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES)
target_compile_definitions(modules__formal_eskf PRIVATE ${FORMAL_ESKF_ALGORITHMS})
# Override PX4's relaxed floating-point options for this numerical core only.
target_compile_options(modules__formal_eskf PRIVATE -fno-fast-math -ffp-contract=off -fstack-usage -fno-exceptions -fno-rtti)
# Scalar domain checks intentionally distinguish exact zero, including signed zero.
target_compile_options(modules__formal_eskf PRIVATE -Wno-float-equal)

# Opt-in host tests run the real module/uORB with controlled time and timer dispatch.
option(FORMAL_ESKF_MODULE_TESTS "Build formal-eskf PX4 module behavior tests" OFF)
if(FORMAL_ESKF_MODULE_TESTS)
	add_executable(formal_eskf_module_test EXCLUDE_FROM_ALL tests/ModuleTest.cpp)
	target_include_directories(formal_eskf_module_test PRIVATE
		"${CMAKE_CURRENT_SOURCE_DIR}"
		"${CMAKE_CURRENT_SOURCE_DIR}/core/include")
	set_target_properties(formal_eskf_module_test PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES)
	target_compile_definitions(formal_eskf_module_test PRIVATE MODULE_NAME="formal_eskf_test"
		${FORMAL_ESKF_ALGORITHMS})
	target_compile_options(formal_eskf_module_test PRIVATE -fno-fast-math -ffp-contract=off
		-fno-exceptions -fno-rtti -Wno-float-equal)
	target_link_libraries(formal_eskf_module_test PRIVATE modules__formal_eskf
		px4_layer px4_platform uORB systemlib cdev px4_work_queue px4_daemon
		work_queue parameters events perf tinybson uorb_msgs geo world_magnetic_model)
	target_link_options(formal_eskf_module_test PRIVATE
		-Wl,--wrap=hrt_absolute_time -Wl,--wrap=hrt_call_after -Wl,--wrap=hrt_cancel
		-Wl,--wrap=pthread_mutex_init
		-Wl,--wrap=pthread_mutex_lock
		-Wl,--wrap=pthread_mutex_trylock
		-Wl,--wrap=_ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv
		-Wl,--wrap=_ZnwmRKSt9nothrow_t
		-Wl,--wrap=perf_alloc)
	enable_testing()
	add_test(NAME formal_eskf.module COMMAND formal_eskf_module_test)
endif()
