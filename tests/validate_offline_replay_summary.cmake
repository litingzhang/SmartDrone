if(NOT DEFINED SUMMARY_JSON)
  message(FATAL_ERROR "SUMMARY_JSON is required")
endif()

string(REGEX REPLACE "^\"(.*)\"$" "\\1" SUMMARY_JSON "${SUMMARY_JSON}")

if(NOT EXISTS "${SUMMARY_JSON}")
  message(FATAL_ERROR "summary json does not exist: ${SUMMARY_JSON}")
endif()

file(READ "${SUMMARY_JSON}" SUMMARY_TEXT)

function(extract_json_int key out_var)
  string(REGEX MATCH "\"${key}\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" MATCH_RESULT "${SUMMARY_TEXT}")
  if(NOT MATCH_RESULT)
    message(FATAL_ERROR "missing integer key '${key}' in ${SUMMARY_JSON}")
  endif()
  set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
endfunction()

extract_json_int("frames_out" FRAMES_OUT)
extract_json_int("tracking_ok_frames" TRACKING_OK_FRAMES)
extract_json_int("identity_pose_frames" IDENTITY_POSE_FRAMES)

if(NOT TRACKING_OK_FRAMES EQUAL FRAMES_OUT)
  message(FATAL_ERROR
          "stereo-only replay baseline failed: tracking_ok_frames=${TRACKING_OK_FRAMES}, frames_out=${FRAMES_OUT}")
endif()

if(IDENTITY_POSE_FRAMES GREATER 1)
  message(FATAL_ERROR
          "stereo-only replay baseline failed: identity_pose_frames=${IDENTITY_POSE_FRAMES}, expected <= 1")
endif()

message(STATUS
        "offline replay stereo-only baseline passed: frames_out=${FRAMES_OUT}, tracking_ok_frames=${TRACKING_OK_FRAMES}, identity_pose_frames=${IDENTITY_POSE_FRAMES}")
