cmake_minimum_required(VERSION 2.8)
project(dynamic_reconfigure)

find_package(catkin REQUIRED)
catkin_package(LIBRARIES dynamic_reconfigure_config_init_mutex
               INCLUDE_DIRS include
               CFG_EXTRAS extras.cmake
)

catkin_python_setup()

find_package(catkin COMPONENTS genmsg
                               roscpp 
                               std_msgs
)
find_package(Boost REQUIRED)


include_directories(include)
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})

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
target_link_libraries(dynamic_reconfigure_config_init_mutex ${BOOST_LIBRARIES})

install(DIRECTORY include/dynamic_reconfigure/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
      
install(TARGETS dynamic_reconfigure_config_init_mutex
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# deal with the CMake folder
file(COPY cmake/rosbuild2.cmake
     DESTINATION ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/
)
file(COPY templates
     DESTINATION ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
      
install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY templates
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
