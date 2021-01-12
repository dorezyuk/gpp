#include <gpp_update_map/gpp_update_map.hpp>
#include <gtest/gtest.h>

// load the plugin with the nav_core interface
TEST(LoadPlugin, generic) {
  pluginlib::ClassLoader<gpp_interface::PrePlanningInterface> loader(
      "gpp_interface", "gpp_interface::PrePlanningInterface");

  // load the plugin
  auto plugin = loader.createInstance("gpp_update_map::GppUpdateMap");
}

int
main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
