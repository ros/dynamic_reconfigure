@[if DEVELSPACE]@
# base dir in develspace
set(dynamic_reconfigure_BASE_DIR @(CMAKE_CURRENT_SOURCE_DIR))
@[else]@
# base dir in installspace
set(dynamic_reconfigure_BASE_DIR @(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION))
@[end if]@

macro(generate_dynamic_reconfigure_options)
  set(_autogen "")
  foreach(_cfg ${ARGN})
    set(_cmd ${CATKIN_ENV}
      ${PROJECT_SOURCE_DIR}/${_cfg}
      ${dynamic_reconfigure_BASE_DIR}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
    debug_message(2 "dynamic reconfigure cmd: ${_cmd}")
    execute_process(COMMAND ${_cmd}
                    RESULT_VARIABLE RES_VAR
                    OUTPUT_VARIABLE OUT_VAR
                    ERROR_VARIABLE ERR_VAR
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_STRIP_TRAILING_WHITESPACE
    )
    if(${RES_VAR} OR NOT "${ERR_VAR}" STREQUAL "")
      message(FATAL_ERROR "Could not run dynamic reconfigure file '${_cfg}': ${ERR_VAR}")
    endif()
    message(STATUS "dynamic_reconfigure built ${_cfg}: ${OUT_VAR}")
  endforeach(_cfg)

  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
endmacro()
