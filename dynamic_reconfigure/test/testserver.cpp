#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/TestConfig.h>

void callback(dynamic_reconfigure::TestConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i %f %s %i %i", config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level);
  
  config.int_ |= 1;
  config.double_ = -config.double_;
  config.str_ += "A";
  config.bool_ = !config.bool_;
  config.level = level;

  ROS_INFO("Reconfigured to     : %i %f %s %i %i", config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_test_server");
  dynamic_reconfigure::Server<dynamic_reconfigure::TestConfig> srv;
  dynamic_reconfigure::Server<dynamic_reconfigure::TestConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO("Constants are: %i %f %s %i", dynamic_reconfigure::Test_int_const, dynamic_reconfigure::Test_double_const, dynamic_reconfigure::Test_str_const, (int) dynamic_reconfigure::Test_bool_const);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
