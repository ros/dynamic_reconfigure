macro(rosbuild_cfgs)
  set(_autogen "")
  foreach(_cfg ${ARGN})

    # Construct the path to the .cfg file
    set(_input ${PROJECT_SOURCE_DIR}/${_cfg})
    
    # rosbuild_gendeps(${PROJECT_NAME} ${_cfg})

    # The .cfg file is its own generator.
    set(gencfg_build_files 
      ${dynamic_reconfigure_SOURCE_DIR}/templates/ConfigType.py
      ${dynamic_reconfigure_SOURCE_DIR}/templates/ConfigType.h
      ${dynamic_reconfigure_SOURCE_DIR}/src/dynamic_reconfigure/parameter_generator.py)

    get_filename_component(_cfgonly ${_cfg} NAME_WE)
    set(_output_cpp ${CMAKE_BINARY_DIR}/gen/cpp/${PROJECT_NAME}/${_cfgonly}Config.h)
    set(_output_py ${CMAKE_BINARY_DIR}/gen/py/${PROJECT_NAME}/cfg/${_cfgonly}Config.py)

    #set(_output_dox ${PROJECT_BINARY_DIR}/docs/${_cfg_bare}Config.dox)
    #set(_output_wikidoc ${PROJECT_BINARY_DIR}/docs/${_cfg_bare}Config.wikidoc)
    #set(_output_usage ${PROJECT_BINARY_DIR}/docs/${_cfg_bare}Config-usage.dox)

    # Add the rule to build the .h the .cfg and the .msg
    # FIXME Horrible hack. Can't get CMAKE to add dependencies for anything
    # but the first output in add_custom_command.
    #    execute_process(
    #      COMMAND ${dynamic_reconfigure_SOURCE_DIR}/cmake/gendeps ${_input}
    #      OUTPUT_VARIABLE __gencfg_autodeps
    #      OUTPUT_STRIP_TRAILING_WHITESPACE)
    # string(REPLACE "\n" " " ${_input}_AUTODEPS ${__gencfg_autodeps})
    # separate_arguments(${_input}_AUTODEPS)
    #message("MSG: " ${${_input}_AUTODEPS})

    add_custom_command(OUTPUT ${_output_cpp} ${_output_py} 
      # ${_output_dox} ${_output_usage} ${_output_py} ${_output_wikidoc}
      COMMAND ${ROSBUILD_SUBSHELL}
      ${_input}
      ${dynamic_reconfigure_SOURCE_DIR}
      ${PROJECT_BINARY_DIR}
      ${ROSBUILD_GEN_DIR}/cpp/${PROJECT_NAME}
      ${ROSBUILD_GEN_DIR}/py/${PROJECT_NAME}
      DEPENDS ${_input} ${gencfg_build_files} rospackexe
      COMMENT "Generating dynamic reconfigure stuff from ${_cfg}: ${_output_cpp} ${_output_py}"
      )

    list(APPEND ${PROJECT_NAME}_generated
      ${_output_cpp} ${_output_py}
      )

  endforeach(_cfg)

  add_custom_target(${PROJECT_NAME}_cfggen
    DEPENDS ${${PROJECT_NAME}_generated})

  add_dependencies(${PROJECT_NAME}_codegen
    ${PROJECT_NAME}_cfggen)

  foreach(_dep ${${PROJECT_NAME}_RECURSIVE_DEPENDS})
    add_dependencies(${PROJECT_NAME}_cfggen ${_dep}_codegen)
  endforeach()
      
endmacro()

