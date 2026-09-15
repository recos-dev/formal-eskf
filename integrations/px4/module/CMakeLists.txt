# install.py copies the core into this module. No source checkout is needed.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/core/include/formal_eskf/eskf/eskf.hpp")
	message(FATAL_ERROR "Install the complete formal-eskf PX4 package, including core/include")
endif()

px4_add_module(
	MODULE modules__formal_eskf
	MAIN formal_eskf
	SRCS FormalEskf.cpp
	DEPENDS px4_work_queue geo world_magnetic_model
)
target_include_directories(modules__formal_eskf PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/core/include")
set_target_properties(modules__formal_eskf PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES)
target_compile_definitions(modules__formal_eskf PRIVATE ESKF_QUAT_APPROX=0 ESKF_RESET_APPROX=0)
# Override PX4's relaxed floating-point options for this numerical core only.
target_compile_options(modules__formal_eskf PRIVATE -fno-fast-math -ffp-contract=off -fstack-usage -fno-exceptions -fno-rtti)
