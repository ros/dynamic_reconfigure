if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
rosbuild_add_library(dynamic_reconfigure_config_init_mutex src/dynamic_reconfigure_config_init_mutex.cpp)
add_subdirectory(test)

