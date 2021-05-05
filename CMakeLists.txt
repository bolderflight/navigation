cmake_minimum_required(VERSION 3.13)
# Project information
project(Navigation
  VERSION 2.0.1
  DESCRIPTION "Navigation filters and utilities"
  LANGUAGES CXX
)
include(FetchContent)
# Fetch eigen
FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://github.com/bolderflight/eigen.git
	GIT_TAG	v2.0.0
)
FetchContent_MakeAvailable(eigen)
#Fetch units
FetchContent_Declare(
	units
	GIT_REPOSITORY 	https://github.com/bolderflight/units.git
	GIT_TAG v3.2.0
)
FetchContent_MakeAvailable(units)
# Add the library target
add_library(navigation
	src/navigation/ekf_15_state.cc
	src/navigation/transforms.cc
	src/navigation/tilt_compass.cc
	src/navigation/utils.cc
	include/navigation/constants.h
	include/navigation/ekf_15_state.h
	include/navigation/tilt_compass.h
	include/navigation/transforms.h
	include/navigation/utils.h
)
target_link_libraries(navigation
	PUBLIC
		eigen
		units
)
# Setup include directories 
target_include_directories(navigation PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

# Example and unit testing if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	# Add the example target
	add_executable(nav_example examples/nav_example.cc)
	# Add the includes
	target_include_directories(nav_example PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
	# Link libraries to the test target
	target_link_libraries(nav_example
		PRIVATE 
			navigation
	)
endif()
