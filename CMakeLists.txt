cmake_minimum_required(VERSION 3.13)
# Project information
project(Navigation
  VERSION 5.0.0
  DESCRIPTION "Navigation filters and utilities"
  LANGUAGES CXX
)
# Fetch eigen
include(FetchContent)
FetchContent_Declare(
	eigen
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/eigen.git
	GIT_TAG					v1.0.0
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
	types
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/types.git
)
FetchContent_MakeAvailable(types)
FetchContent_Declare(
	global_defs
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/global_defs.git
)
FetchContent_MakeAvailable(global_defs)
# Add the library target
add_library(nav INTERFACE)
target_link_libraries(nav
	INTERFACE
		eigen
		types
		global_defs
)
# Setup include directories 
target_include_directories(nav INTERFACE include/)
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
			nav
	)
# 	# Fetch google test
# 	include(FetchContent)
# 	FetchContent_Declare(
# 		googletest
# 		GIT_REPOSITORY 	https://github.com/google/googletest.git
# 		GIT_TAG					release-1.10.0
# 	)
# 	FetchContent_MakeAvailable(googletest)
# 	enable_testing()
# 	include(GoogleTest)
# 	# Add the test target
# 	add_executable(types_test tests/types_test.cc)
# 	# Add the includes
# 	target_include_directories(types_test PUBLIC 
# 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 		$<INSTALL_INTERFACE:include>
# 	)
# 	# Link libraries to the test target
# 	target_link_libraries(types_test
# 		PRIVATE 
# 			types
# 			gtest_main 
# 			gtest
# 			gmock
# 	)
# 	# Discover unit tests
# 	gtest_discover_tests(types_test)
endif()
