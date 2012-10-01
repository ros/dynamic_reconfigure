get_filename_component(dynamic_reconfigure_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

macro(generate_cfg)
  set(_autogen "")
  foreach(_cfg ${ARGN})

    # Construct the path to the .cfg file
    set(_input ${PROJECT_SOURCE_DIR}/${_cfg})

    # The .cfg file is its own generator.
    set(gencfg_build_files 
      ${dynamic_reconfigure_SELF_DIR}/../templates/ConfigType.py
      ${dynamic_reconfigure_SELF_DIR}/../templates/ConfigType.h
    )

    get_filename_component(_cfgonly ${_cfg} NAME_WE)
    set(_output_cpp ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Config.h)
    set(_output_py ${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/${_cfgonly}Config.py)
    set(_output_dox ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config.dox)
    set(_output_wikidoc ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config.wikidoc)
    set(_output_usage ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfg_bare}Config-usage.dox)

message(INFO ${_output_cpp} ${_output_dox} ${_output_usage} ${_output_py} ${_output_wikidoc})
    add_custom_command(OUTPUT
      ${_output_cpp} ${_output_dox} ${_output_usage} ${_output_py} ${_output_wikidoc}
      COMMAND ${CATKIN_ENV}
      ${_input}
      ${dynamic_reconfigure_SELF_DIR}/..
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
      DEPENDS ${_input} ${gencfg_build_files}
      COMMENT "Generating dynamic reconfigure stuff from ${_cfg}: ${_output_cpp} ${_output_py}"
      )

    list(APPEND ${PROJECT_NAME}_generated
      ${_output_cpp} ${_output_py}
      )
  endforeach(_cfg)

  add_custom_target(${PROJECT_NAME}_gencfg DEPENDS ${${PROJECT_NAME}_generated})
endmacro()
