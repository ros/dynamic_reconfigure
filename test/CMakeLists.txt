add_executable(testserver testserver.cpp testserver2.cpp)
add_dependencies(testserver ${PROJECT_NAME}_gencfg)
target_link_libraries(testserver pthread dynamic_reconfigure_config_init_mutex ${catkin_LIBRARIES})
add_rostest(test_cpp.xml)
