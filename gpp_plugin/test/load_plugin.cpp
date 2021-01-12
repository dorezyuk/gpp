#include <gpp_plugin/gpp_plugin.hpp>
#include <gtest/gtest.h>

// load the plugin with the nav_core interface
TEST(LoadPlugin, NavCore) {
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> loader(
      "nav_core", "nav_core::BaseGlobalPlanner");

  // load the plugin
  auto plugin =
      loader.createInstance("gpp_plugin::GppPlugin");
}

// load the plugin with mbf interface
TEST(LoadPlugin, MbfCostmapCore) {
  pluginlib::ClassLoader<mbf_costmap_core::CostmapPlanner> loader(
      "mbf_costmap_core", "mbf_costmap_core::CostmapPlanner");

  // load the plugin
  auto plugin =
      loader.createInstance("gpp_plugin::GppPlugin");
}

int
main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
