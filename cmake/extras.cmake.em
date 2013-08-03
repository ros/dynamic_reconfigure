@[if DEVELSPACE]@
# base dir in develspace
set(dynamic_reconfigure_BASE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)")
@[else]@
# base dir in installspace
set(dynamic_reconfigure_BASE_DIR "${dynamic_reconfigure_DIR}/..")
@[end if]@

macro(generate_dynamic_reconfigure_options)
  if(${PROJECT_NAME}_CATKIN_PACKAGE)
    message(FATAL_ERROR "generate_dynamic_reconfigure_options() must be called before catkin_package() in project '${PROJECT_NAME}'")
  endif()

  # ensure that package destination variables are defined
  catkin_destinations()

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
    set(_output_dox ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config.dox)
    set(_output_wikidoc ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config.wikidoc)
    set(_output_usage ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config-usage.dox) 

    assert(CATKIN_ENV)
    set(_cmd ${CATKIN_ENV}
      ${_input}
      ${dynamic_reconfigure_BASE_DIR}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  endforeach(_cfg)

  # gencfg target for hard dependency on dynamic_reconfigure generation
  add_custom_target(${PROJECT_NAME}_gencfg DEPENDS ${${PROJECT_NAME}_generated})

  # register target for catkin_package(EXPORTED_TARGETS)
  list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_gencfg)

  dynreconf_called()
endmacro()

macro(dynreconf_called)
  if(NOT dynamic_reconfigure_CALLED)
    set(dynamic_reconfigure_CALLED TRUE)

    # mark that generate_dynamic_reconfigure_options() was called in order to detect wrong order of calling with catkin_python_setup()
    set(${PROJECT_NAME}_GENERATE_DYNAMIC_RECONFIGURE TRUE)
    # check if catkin_python_setup() was called in order to skip installation of generated __init__.py file
    set(package_has_static_sources ${${PROJECT_NAME}_CATKIN_PYTHON_SETUP})

    # generate empty __init__ to make parent folder of msg/srv a python module
    if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py)
      file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py "")
    endif()
    if(NOT package_has_static_sources)
      # install package __init__.py
      install(
        FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
      )
    endif()

    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

    # generate cfg module __init__.py
    if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py)
      file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py "")
    endif()

    # compile python code before installing
    find_package(PythonInterp REQUIRED)
    install(CODE "execute_process(COMMAND \"${PYTHON_EXECUTABLE}\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg\")")
    install(
      DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg
      DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif()
endmacro()
