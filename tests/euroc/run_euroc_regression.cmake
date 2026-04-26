if(NOT DEFINED REPLAY_EXE)
  message(FATAL_ERROR "REPLAY_EXE is required")
endif()
if(NOT DEFINED PYTHON_EXECUTABLE)
  message(FATAL_ERROR "PYTHON_EXECUTABLE is required")
endif()
if(NOT DEFINED EVALUATOR)
  message(FATAL_ERROR "EVALUATOR is required")
endif()
if(NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "OUTPUT_DIR is required")
endif()
if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(EUROC_DATASET "$ENV{SMART_DRONE_EUROC_DATASET}")
if(EUROC_DATASET STREQUAL "")
  message(STATUS "SMART_DRONE_EUROC_DATASET is not set; skipping EuRoC regression")
  return()
endif()

if(PYTHON_EXECUTABLE STREQUAL "" OR NOT EXISTS "${PYTHON_EXECUTABLE}")
  message(STATUS "Python3 interpreter not available; skipping EuRoC regression")
  return()
endif()

if(NOT EXISTS "${EUROC_DATASET}")
  message(FATAL_ERROR "SMART_DRONE_EUROC_DATASET does not exist: ${EUROC_DATASET}")
endif()

set(MAX_FRAMES "$ENV{SMART_DRONE_EUROC_MAX_FRAMES}")
if(MAX_FRAMES STREQUAL "")
  set(MAX_FRAMES "600")
endif()

set(MAX_ATE_RMSE "$ENV{SMART_DRONE_EUROC_MAX_ATE_RMSE}")
if(MAX_ATE_RMSE STREQUAL "")
  set(MAX_ATE_RMSE "2.5")
endif()

set(MAX_RPE_RMSE "$ENV{SMART_DRONE_EUROC_MAX_RPE_RMSE}")
if(MAX_RPE_RMSE STREQUAL "")
  set(MAX_RPE_RMSE "1.0")
endif()

set(FEATURE_FRONTEND "$ENV{SMART_DRONE_EUROC_FEATURE_FRONTEND}")
if(FEATURE_FRONTEND STREQUAL "")
  set(FEATURE_FRONTEND "orb")
endif()

file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(POSE_CSV "${OUTPUT_DIR}/euroc_pose.csv")
set(SUMMARY_JSON "${OUTPUT_DIR}/euroc_summary.json")
set(METRICS_JSON "${OUTPUT_DIR}/euroc_metrics.json")

execute_process(
  COMMAND "${REPLAY_EXE}"
          --dataset "${EUROC_DATASET}"
          --stereo-only
          --feature-frontend "${FEATURE_FRONTEND}"
          --max-frames "${MAX_FRAMES}"
          --out "${POSE_CSV}"
          --summary-json "${SUMMARY_JSON}"
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE REPLAY_RESULT
)
if(NOT REPLAY_RESULT EQUAL 0)
  message(FATAL_ERROR "EuRoC offline replay failed: ${REPLAY_RESULT}")
endif()

execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" "${EVALUATOR}"
          --dataset "${EUROC_DATASET}"
          --estimate "${POSE_CSV}"
          --out-json "${METRICS_JSON}"
          --max-ate-rmse "${MAX_ATE_RMSE}"
          --max-rpe-trans-rmse "${MAX_RPE_RMSE}"
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE EVAL_RESULT
)
if(NOT EVAL_RESULT EQUAL 0)
  message(FATAL_ERROR "EuRoC trajectory regression failed: ${EVAL_RESULT}")
endif()

message(STATUS "EuRoC regression passed; metrics: ${METRICS_JSON}")
