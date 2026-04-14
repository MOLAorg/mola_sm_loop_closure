# Unit tests:

find_package(ament_cmake_gtest REQUIRED)
find_package(mola_test_datasets)

if(NOT mola_test_datasets_FOUND)
  message(STATUS "**********************************************************************")
  message(STATUS " WARNING: Skipping unit tests since mola_test_datasets was not found  ")
  message(STATUS "**********************************************************************")

  # If we are in a ROS build farm, unit tests are MANDATORY, not like in user local builds:
  if(("${BUILD_TESTING}" STREQUAL "1") AND ("$ENV{HOME}" STREQUAL "/home/buildfarm"))
    message(SEND_ERROR "** Requirements for unit tests not found (see messages above). Stopping generation since unit tests are required in build farms **")
  endif()

  return()
endif()

set(LC_PIPELINE_YAML ${mola_sm_loop_closure_SOURCE_DIR}/pipelines/loop-closure-f2f-lidar3d-gicp.yaml)
set(WAREHOUSE_SIMPLEMAP "${mola_test_datasets_DIR}/../datasets/simplemaps/mvsim-warehouse01.simplemap")
set(OUTPUT_SIMPLEMAP "${CMAKE_CURRENT_BINARY_DIR}/tmp-warehouse-test.simplemap")

ament_add_gtest(test_cli_f2f_gicp test_cli_f2f_gicp.cpp
  ENV LC_PIPELINE_YAML=${LC_PIPELINE_YAML}
  ENV LC_INPUT_SIMPLEMAP=${WAREHOUSE_SIMPLEMAP}
  ENV LC_OUTPUT_SIMPLEMAP=${OUTPUT_SIMPLEMAP}
  ENV MOLA_DESKEW_METHOD=MotionCompensationMethod::None
  TIMEOUT 300
)
target_link_libraries(test_cli_f2f_gicp)
