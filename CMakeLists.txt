# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_sm_loop_closure LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mola_pose_list REQUIRED)

# find dependencies:
find_package(mrpt-maps REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-tclap REQUIRED)

find_package(GTSAM REQUIRED)

find_package(mp2p_icp REQUIRED)
find_package(mp2p_icp_filters REQUIRED)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS module/src/*.cpp module/src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS module/include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::maps
		mrpt::gui
		mola::mp2p_icp
		mola::mp2p_icp_filters
		mola::mola_yaml
		mola::mola_pose_list
	PRIVATE_LINK_LIBRARIES
		gtsam
	CMAKE_DEPENDENCIES
		mp2p_icp
		mp2p_icp_filters
		mola_pose_list
		mrpt-maps
)

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

mola_add_executable(
	TARGET  mola-sm-georeferencing
	SOURCES apps/mola-sm-georeferencing-cli.cpp
	LINK_LIBRARIES
		${PROJECT_NAME}
		mrpt::maps
		mrpt::tclap
)

# -----------------------
# define tests:
#enable_testing()
#add_subdirectory(module/tests)

# Install files:
#install(DIRECTORY mola-cli-launchs DESTINATION share/${PROJECT_NAME})

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	ament_package()
endif()
