# Unit tests:

find_package(ament_cmake_gtest REQUIRED)

# Pure unit tests, no datasets needed:
ament_add_gtest(test_gnss_alignment_and_densify test_gnss_alignment_and_densify.cpp)
target_link_libraries(test_gnss_alignment_and_densify ${PROJECT_NAME})
find_package(mola_test_datasets)

# Global registration: synthetic cases, plus the loop closure cases of
# mola_test_datasets. If the installed version lacks any of these sets (older
# release), they are fetched from its repository (shallow, only that directory).
# Without them (e.g. no network), the dataset test is skipped.
set(_LC_CASE_SETS vbr husky kitti)
set(_LC_CASES_DIR "")

function(_lc_cases_complete dir out)
  set(ok TRUE)
  foreach(s ${_LC_CASE_SETS})
    if(NOT EXISTS "${dir}/${s}/cases.yaml")
      set(ok FALSE)
    endif()
  endforeach()
  set(${out} ${ok} PARENT_SCOPE)
endfunction()

if(mola_test_datasets_FOUND)
  set(_dir "${mola_test_datasets_DIR}/../datasets/loop_closures")
  _lc_cases_complete("${_dir}" _complete)
  if(_complete)
    set(_LC_CASES_DIR "${_dir}")
  endif()
endif()

if(NOT _LC_CASES_DIR)
  find_package(Git QUIET)
  set(_clone "${CMAKE_CURRENT_BINARY_DIR}/mola_test_datasets")
  set(_url "https://github.com/MOLAorg/mola_test_datasets.git")
  _lc_cases_complete("${_clone}/datasets/loop_closures" _complete)
  if(GIT_FOUND AND NOT _complete)
    message(STATUS "Fetching the loop closure cases of mola_test_datasets")
    if(NOT EXISTS "${_clone}/.git")
      execute_process(
        COMMAND ${GIT_EXECUTABLE} clone --depth 1 --filter=blob:none --sparse
                ${_url} "${_clone}"
        OUTPUT_QUIET ERROR_QUIET TIMEOUT 300)
    else()
      execute_process(
        COMMAND ${GIT_EXECUTABLE} -C "${_clone}" pull --depth 1
        OUTPUT_QUIET ERROR_QUIET TIMEOUT 300)
    endif()
    execute_process(
      COMMAND ${GIT_EXECUTABLE} -C "${_clone}"
              sparse-checkout set datasets/loop_closures
      OUTPUT_QUIET ERROR_QUIET TIMEOUT 300)
    _lc_cases_complete("${_clone}/datasets/loop_closures" _complete)
  endif()
  if(_complete)
    set(_LC_CASES_DIR "${_clone}/datasets/loop_closures")
  else()
    message(WARNING
      "Loop closure cases of mola_test_datasets not available: test skipped")
  endif()
endif()

ament_add_gtest(test_bev_correlation test_bev_correlation.cpp
  ENV LC_CASES_DIR=${_LC_CASES_DIR}
  TIMEOUT 300
)
target_link_libraries(test_bev_correlation ${PROJECT_NAME})

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

# The CLI from this build tree, not from PATH: a Debian package build runs the tests before
# installing, so mola-sm-lc-cli is not on PATH there.
ament_add_gtest(test_cli_f2f_gicp test_cli_f2f_gicp.cpp
  ENV LC_CLI=$<TARGET_FILE:mola-sm-lc-cli>
  ENV LC_PIPELINE_YAML=${LC_PIPELINE_YAML}
  ENV LC_INPUT_SIMPLEMAP=${WAREHOUSE_SIMPLEMAP}
  ENV LC_OUTPUT_SIMPLEMAP=${OUTPUT_SIMPLEMAP}
  ENV MOLA_DESKEW_METHOD=MotionCompensationMethod::None
  TIMEOUT 300
)
target_link_libraries(test_cli_f2f_gicp)

# Candidate admission + edge correctness against the fixture's own poses:
ament_add_gtest(test_lc_candidates test_lc_candidates.cpp
  ENV LC_PIPELINE_YAML=${LC_PIPELINE_YAML}
  ENV LC_INPUT_SIMPLEMAP=${WAREHOUSE_SIMPLEMAP}
  ENV MOLA_DESKEW_METHOD=MotionCompensationMethod::None
  TIMEOUT 300
)
target_link_libraries(test_lc_candidates ${PROJECT_NAME})

# Direct API test for the detector-only analyze() path:
ament_add_gtest(test_analyze_f2f test_analyze_f2f.cpp
  ENV LC_PIPELINE_YAML=${LC_PIPELINE_YAML}
  ENV LC_INPUT_SIMPLEMAP=${WAREHOUSE_SIMPLEMAP}
  ENV MOLA_DESKEW_METHOD=MotionCompensationMethod::None
  TIMEOUT 300
)
target_link_libraries(test_analyze_f2f ${PROJECT_NAME})
