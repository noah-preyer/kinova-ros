#include <gtest/gtest.h>

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

// Only instantiates the plugin: on_init is not called, so no device is ever created.
TEST(KinovaSystemPlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  std::shared_ptr<hardware_interface::SystemInterface> system;
  ASSERT_NO_THROW(system = loader.createSharedInstance("kinova_driver/KinovaSystem"));
  EXPECT_NE(system, nullptr);
}
