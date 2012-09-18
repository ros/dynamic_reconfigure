cmake_minimum_required(VERSION 2.8)
project(dynamic_reconfigure)

find_package(catkin REQUIRED)
catkin_stack()
catkin_project(dynamic_reconfigure
  LIBRARIES dynamic_reconfigure_config_init_mutex
  INCLUDE_DIRS include)


find_package(catkin COMPONENTS
  rostime cpp_common roscpp_serialization roscpp_traits # serialization
  roscpp rosconsole                                     # roscpp
  std_msgs
  genmsg)

catkin_python_setup()

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

find_package(Boost REQUIRED)
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
add_library(dynamic_reconfigure_config_init_mutex SHARED
  src/dynamic_reconfigure_config_init_mutex.cpp)
target_link_libraries(dynamic_reconfigure_config_init_mutex ${BOOST_LIBRARIES})

install(FILES manifest.xml
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION})

install(DIRECTORY include/dynamic_reconfigure/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
      
install(TARGETS dynamic_reconfigure_config_init_mutex
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})
      
install(PROGRAMS scripts/dynparam
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/scripts/)
