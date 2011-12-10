cmake_minimum_required(VERSION 2.8)
project(dynamic_reconfigure)

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
  ConfigDescription.msg  DoubleParameter.msg  GroupState.msg  ParamDescription.msg  StrParameter.msg
)

add_service_files(
  DIRECTORY srv
  FILES Reconfigure.srv)

generate_messages()

add_library(dynamic_reconfigure_config_init_mutex src/dynamic_reconfigure_config_init_mutex.cpp)

install_cmake_infrastructure(dynamic_reconfigure
  VERSION 0.0.1
  INCLUDE_DIRS include
  )


catkin_package(dynamic_reconfigure)
enable_python(dynamic_reconfigure)
