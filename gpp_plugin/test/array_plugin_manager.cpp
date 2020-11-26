#include <gpp_plugin/gpp_plugin.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace gpp_plugin;

TEST(ArrayPluginManagerTest, Loading) {
  GlobalPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("plugins", nh);
}

int
main(int argc, char **argv) {
  ros::init(argc, argv, "array_plugin_manager");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}