@[if BUILDSPACE]@
# base dir in buildspace
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
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
    message("dynconf cmd: ${_cmd}")
    execute_process(COMMAND ${_cmd}
                    RESULT_VARIABLE RES_VAR
                    OUTPUT_VARIABLE OUT_VAR
                    ERROR_VARIABLE ERR_VAR
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "dynamic_reconfigure building ${_cfg}")
    message(STATUS "dynamic_reconfigure built ${_cfg}: " + ${OUT_VAR})
    if (${ERR_VAR})
      message(ERROR ${ERR_VAR})
    endif()
  endforeach(_cfg)

  include_directories(${CATKIN_BUILD_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
endmacro()
