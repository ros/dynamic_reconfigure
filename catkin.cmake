cmake_minimum_required(VERSION 2.8)
project(dynamic_reconfigure)
catkin_project(dynamic_reconfigure
  LIBRARIES dynamic_reconfigure_config_init_mutex
  INCLUDE_DIRS include)

find_package(catkin REQUIRED)
find_package(ROS COMPONENTS
  rostime cpp_common roscpp_serialization roscpp_traits # serialization
  roscpp rosconsole                                     # roscpp
  std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES 
  BoolParameter.msg      Config.msg           Group.msg       IntParameter.msg      SensorLevels.msg
  ConfigDescription.msg  DoubleParameter.msg  GroupState.msg  ParamDescription.msg  StrParameter.msg)

add_service_files(
  DIRECTORY srv
  FILES Reconfigure.srv)

generate_messages()

add_library(dynamic_reconfigure_config_init_mutex SHARED
  src/dynamic_reconfigure_config_init_mutex.cpp)

catkin_export_python(src)

install(FILES manifest.xml
  DESTINATION share/dynamic_reconfigure)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")
      
install(TARGETS dynamic_reconfigure_config_init_mutex
  LIBRARY DESTINATION lib)
      
install(PROGRAMS scripts/dynparam scripts/reconfigure_gui
  DESTINATION share/dynamic_reconfigure/)