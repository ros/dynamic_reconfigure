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
    # Construct the path to the .cfg file
    set(_input ${PROJECT_SOURCE_DIR}/${_cfg})
    
    # The .cfg file is its own generator.
    set(gencfg_build_files 
      ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.py
      ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.h
    ) 

    get_filename_component(_cfgonly ${_cfg} NAME_WE)
    set(_output_cpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Config.h)
    set(_output_py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/${_cfgonly}Config.py)
    set(_output_dox ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config.dox)
    set(_output_wikidoc ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config.wikidoc)
    set(_output_usage ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config-usage.dox) 

    set(_cmd ${CATKIN_ENV}
      ${_input}
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

    #file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py)

    add_custom_command(OUTPUT
      ${_output_cpp} ${_output_dox} ${_output_usage} ${_output_py} ${_output_wikidoc}
      COMMAND ${_cmd} 
      DEPENDS ${_input} ${gencfg_build_files}
      COMMENT "Generating dynamic reconfigure files from ${_cfg}: ${_output_cpp} ${_output_py}"
    )


    list(APPEND ${PROJECT_NAME}_generated
      ${_output_cpp} ${_output_py}
    )

    install(FILES ${_output_cpp}
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(FILES ${_output_py}
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg)
  endforeach(_cfg)

  # gencfg target for hard dependency od dynamic_reconfigure generation
  add_custom_target(${PROJECT_NAME}_gencfg DEPENDS ${${PROJECT_NAME}_generated})

  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
endmacro()
