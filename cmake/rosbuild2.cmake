get_filename_component(dynamic_reconfigure_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

macro(generate_cfg)
  set(_autogen "")
  foreach(_cfg ${ARGN})

    # Construct the path to the .cfg file
    set(_input ${PROJECT_SOURCE_DIR}/${_cfg})

    execute_process(COMMAND ${CATKIN_ENV}
                            ${_input}
                            ${dynamic_reconfigure_SELF_DIR}/..
                            ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
                            ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                            ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
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
