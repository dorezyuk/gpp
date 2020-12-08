#include <gpp_plugin/gpp_plugin.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace gpp_plugin;

// currently disabled, since global-planner will segfault without calling init
// see https://github.com/ros-planning/navigation/issues/1026
TEST(ArrayPluginManagerTest, DISABLED_Loading) {
  // here we check that we can properly load plugins, even if some are invalid
  GlobalPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("plugins", nh);

  // check the size of the array
  EXPECT_EQ(manager.getPlugins().size(), 3);
}

TEST(CostmapPlannerManagerTest, Loading) {
  // same as above, but with the CostmapPluginManager
  CostmapPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("plugins", nh);

  // check the size of the array
  EXPECT_EQ(manager.getPlugins().size(), 3);
}

TEST(ArrayPluginManagerTest, NotAnArray) {
  // here we check that we survive really bad user configs
  GlobalPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("not_an_array", nh);

  // nothing to load here
  EXPECT_TRUE(manager.getPlugins().empty());
}

TEST(ArrayPluginManagerTest, Missing) {
  // here we check that we survive if the user forgets to define the parameter
  GlobalPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("missing", nh);

  // nothing to load here
  EXPECT_TRUE(manager.getPlugins().empty());
}

int
main(int argc, char** argv) {
  ros::init(argc, argv, "array_plugin_manager");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}