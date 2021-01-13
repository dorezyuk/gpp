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
  ASSERT_EQ(manager.getPlugins().size(), 3);

  // now check the flags
  const auto& plugins = manager.getPlugins();
  std::array<bool, 3> on_success_break = {false, true, false};
  std::array<bool, 3> on_failure_break = {true, false, true};

  // check the loaded flags
  for (size_t ii = 0; ii != plugins.size(); ++ii) {
    EXPECT_EQ(plugins.at(ii).first.on_success_break, on_success_break.at(ii));
    EXPECT_EQ(plugins.at(ii).first.on_failure_break, on_failure_break.at(ii));
  }
}

TEST(CostmapPlannerManagerTest, DISABLED_Loading) {
  // same as above, but with the CostmapPluginManager
  CostmapPlannerManager manager;
  ros::NodeHandle nh("~");
  manager.load("plugins", nh);

  // check the size of the array
  ASSERT_EQ(manager.getPlugins().size(), 3);

  // now check the flags
  const auto& plugins = manager.getPlugins();
  std::array<bool, 3> on_success_break = {false, true, false};
  std::array<bool, 3> on_failure_break = {true, false, true};

  // check the loaded flags
  for (size_t ii = 0; ii != plugins.size(); ++ii) {
    EXPECT_EQ(plugins.at(ii).first.on_success_break, on_success_break.at(ii));
    EXPECT_EQ(plugins.at(ii).first.on_failure_break, on_failure_break.at(ii));
  }
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
