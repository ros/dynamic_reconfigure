include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

rosbuild_add_library(dynamic_reconfigure_config_init_mutex src/dynamic_reconfigure_config_init_mutex.cpp)
add_subdirectory(test)

