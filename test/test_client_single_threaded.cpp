#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/TestConfig.h>

using namespace dynamic_reconfigure_test;
using namespace dynamic_reconfigure;


TestConfig CONFIG;

TEST(dynamic_reconfigure_client_single_thread, singleThreadedSetGet) {
  Client<TestConfig> client("/ref_server");
  CONFIG = TestConfig::__getDefault__();
  EXPECT_TRUE(client.setConfiguration(CONFIG));
  EXPECT_TRUE(client.getCurrentConfiguration(CONFIG));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_single_threaded_client_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
