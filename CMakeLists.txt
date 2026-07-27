# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                              (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_sm_loop_closure LANGUAGES CXX)

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  add_compile_options(--coverage)
  add_link_options(--coverage)
endif()

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)


# find dependencies:
find_package(GTSAM REQUIRED)
find_package(mola_georeferencing REQUIRED)
find_package(mola_gtsam_factors REQUIRED)
find_package(mola_pose_list REQUIRED)
find_package(mola_relocalization REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mp2p_icp_filters REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)

# -----------------------
# define lib:
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/SimplemapLoopClosure.cpp
    module/src/FrameToFrameLoopClosure.cpp
    module/src/LoopClosureInterface.cpp
    module/src/register.cpp
    module/src/common/obs_helpers.cpp
    module/src/common/planarity_factors.cpp
    module/src/common/gnc_optimizer.cpp
    module/src/common/tum_writer.cpp
    module/src/common/graph_builders.cpp
    module/src/common/icp_pipeline_setup.cpp
    module/src/common/gnss_factor_helpers.cpp
    module/src/common/debug_flags.cpp
    # Headers:
    module/include/mola_sm_loop_closure/SimplemapLoopClosure.h
    module/include/mola_sm_loop_closure/FrameToFrameLoopClosure.h
    module/include/mola_sm_loop_closure/common/obs_helpers.h
    module/include/mola_sm_loop_closure/common/planarity_factors.h
    module/include/mola_sm_loop_closure/common/gnc_optimizer.h
    module/include/mola_sm_loop_closure/common/tum_writer.h
    module/include/mola_sm_loop_closure/common/graph_builders.h
    module/include/mola_sm_loop_closure/common/icp_pipeline_setup.h
    module/include/mola_sm_loop_closure/common/gnss_factor_helpers.h
    module/include/mola_sm_loop_closure/common/debug_flags.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mola::mola_georeferencing
    mola::mola_gtsam_factors
    mola::mola_pose_list
    mola::mola_relocalization
    mola::mola_yaml
    mola::mp2p_icp
    mola::mp2p_icp_filters
    mrpt::maps
#	PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mola_georeferencing
    mola_gtsam_factors
    mola_pose_list
    mola_relocalization
    mp2p_icp
    mp2p_icp_filters
    mrpt-maps
)

# Check mp2p_icp version for log functor support
if(mp2p_icp_VERSION VERSION_GREATER_EQUAL "2.6.0")
  target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_ICP_HAS_LOG_FUNCTOR)
endif()

# Detect whether mola_georeferencing exposes the newer, unified
# extract_imu_frames_from_sm()/IMUFrames API (gravity + absolute attitude), or only the
# older extract_imu_acc_frames_from_sm()/IMUAccFrames one, kept for old installed versions.
# A version check is not reliable here since the new API landed without a version bump,
# so probe for it directly instead.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES mola::mola_georeferencing)
check_cxx_source_compiles(
  "#include <mola_georeferencing/simplemap_georeference.h>
   int main() { auto fn = &mola::extract_imu_frames_from_sm; (void)fn; return 0; }"
  MOLA_GEOREFERENCING_HAS_NEW_IMU_API)
unset(CMAKE_REQUIRED_LIBRARIES)
if(MOLA_GEOREFERENCING_HAS_NEW_IMU_API)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MOLA_GEOREFERENCING_HAS_NEW_IMU_API)
endif()

# Optional: KISS-Matcher for global-registration initial guesses in F2F loop closure.
# Enabled automatically when the git submodule is populated.
set(_KISS_MATCHER_CMAKE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/kiss-matcher/cpp/kiss_matcher/CMakeLists.txt")
set(_ROBIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/robin")
if(EXISTS "${_KISS_MATCHER_CMAKE}" AND EXISTS "${_ROBIN_DIR}")
  message(STATUS "KISS-Matcher submodule found: enabling MOLA_HAS_KISS_MATCHER")
  # Use system packages instead of FetchContent downloads where possible
  set(USE_SYSTEM_EIGEN3 ON  CACHE BOOL "Use system Eigen3 for KISS-Matcher" FORCE)
  set(USE_SYSTEM_TBB    ON  CACHE BOOL "Use system TBB for KISS-Matcher"   FORCE)
  set(USE_SYSTEM_ROBIN  OFF CACHE BOOL "Use system ROBIN for KISS-Matcher"  FORCE)
  # Redirect ROBIN FetchContent to our bundled submodule (avoids network fetches)
  set(FETCHCONTENT_SOURCE_DIR_ROBIN "${_ROBIN_DIR}" CACHE PATH "" FORCE)
  add_subdirectory(
    third_party/kiss-matcher/cpp/kiss_matcher
    ${CMAKE_CURRENT_BINARY_DIR}/third_party/kiss_matcher_build
    EXCLUDE_FROM_ALL
  )
  # Static libs linked into our .so must be compiled with -fPIC
  set_target_properties(kiss_matcher_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set_target_properties(robin            PROPERTIES POSITION_INDEPENDENT_CODE ON)
  # pmc uses -Wno-format (disabling -Wformat), which conflicts with Debian's
  # injected -Wformat-security (which requires -Wformat). Silence it explicitly.
  if(TARGET pmc)
    target_compile_options(pmc PRIVATE -Wno-format-security)
  endif()
  # Silence upstream warnings in third-party sources we don't maintain.
  if(TARGET robin)
    target_compile_options(robin PRIVATE -Wno-return-type)
  endif()
  if(TARGET kiss_matcher_core)
    target_compile_options(kiss_matcher_core PRIVATE -Wno-unused-variable)
  endif()
  # Treat KISS-Matcher's (and dependencies') headers as system headers so
  # their internal warnings (shadowing, macro redefinition, etc.) don't
  # pollute the build of our own code that includes them.
  # Portable across CMake versions (Humble ships CMake 3.22): apply the
  # SYSTEM keyword to our own target's view of their include directories,
  # rather than relying on the SYSTEM target property (CMake >= 3.25 only).
  foreach(_dep_target kiss_matcher_core robin pmc)
    if(TARGET ${_dep_target})
      get_target_property(_dep_includes ${_dep_target} INTERFACE_INCLUDE_DIRECTORIES)
      if(_dep_includes)
        target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${_dep_includes})
      endif()
    endif()
  endforeach()
  unset(_dep_includes)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MOLA_HAS_KISS_MATCHER)
  target_link_libraries(${PROJECT_NAME} PRIVATE kiss_matcher::kiss_matcher_core)
elseif(EXISTS "${_KISS_MATCHER_CMAKE}" AND NOT EXISTS "${_ROBIN_DIR}")
  message(STATUS "KISS-Matcher submodule found but ROBIN submodule is missing "
    "(third_party/robin); skipping MOLA_HAS_KISS_MATCHER. "
    "Run: git submodule update --init third_party/robin")
endif()

# mola_add_library() expects headers under "ROOT/include", so do this manually here:
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# CLI apps:
# ---------------------------------
mola_add_executable(
  TARGET  mola-sm-lc-cli
  SOURCES apps/mola-sm-lc-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
)


# -----------------------
# define tests:
if(BUILD_TESTING)
  enable_testing()
  if (DETECTED_ROS2)
    find_package(ament_cmake REQUIRED)
  endif()
  add_subdirectory(test)
endif()

# Install files:
install(DIRECTORY
  pipelines
  DESTINATION
    share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)  # no-op if already found above
  ament_package()
endif()
