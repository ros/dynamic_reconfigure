#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/TestConfig.h>

void callback(dynamic_reconfigure_test::TestConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure request: %i %f %s %i %i Group1: %i Group2: %f %s",
            config.int_, config.double_, config.str_.c_str(),
            (int) config.bool_, config.level, config.group1_int,
            config.group2_double, config.group2_string.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_reconfigure_test");

  dynamic_reconfigure::Server<dynamic_reconfigure_test::TestConfig> server;
  dynamic_reconfigure::Server<dynamic_reconfigure_test::TestConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
