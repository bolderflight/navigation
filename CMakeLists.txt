cmake_minimum_required(VERSION 3.14)
# Project information
project(Navigation
  VERSION 4.0.1
  DESCRIPTION "Navigation filters and utilities"
  LANGUAGES CXX
)
include(FetchContent)
# Fetch eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://github.com/bolderflight/eigen.git
  GIT_TAG	v3.0.2
)
FetchContent_MakeAvailable(eigen)
#Fetch units
FetchContent_Declare(
  units
  GIT_REPOSITORY 	https://github.com/bolderflight/units.git
  GIT_TAG v4.1.0
)
FetchContent_MakeAvailable(units)
# Add the library target
add_library(navigation INTERFACE)
target_link_libraries(navigation
  INTERFACE
    eigen
    units
)
# Setup include directories 
target_include_directories(navigation INTERFACE src)

# Example and unit testing if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
  # Add the example target
  add_executable(nav_example examples/cmake/nav_example.cc)
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
