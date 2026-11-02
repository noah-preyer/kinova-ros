#include <gtest/gtest.h>

#include <string>

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_comm_config.hpp"

TEST(KinovaApi, InitializeFailsWhenLibrariesMissing) {
  kinova::KinovaAPI api;
  EXPECT_EQ(api.initializeKinovaAPIFunctions(kinova::USB, "/nonexistent_dir"), 0);
  EXPECT_EQ(api.initializeKinovaAPIFunctions(kinova::ETHERNET, "/nonexistent_dir"), 0);
}

TEST(KinovaComm, ConfigDefaults) {
  kinova::KinovaCommConfig config;
  EXPECT_EQ(config.serial_number, "not_set");
  EXPECT_EQ(config.connection_type, "USB");
  EXPECT_EQ(config.local_machine_ip, "192.168.100.100");
  EXPECT_EQ(config.subnet_mask, "255.255.255.0");
  EXPECT_EQ(config.local_cmd_port, 25000);
  EXPECT_EQ(config.local_broadcast_port, 25025);
  EXPECT_FALSE(config.use_estimated_com);
  EXPECT_TRUE(config.movement_on_start);
  EXPECT_TRUE(config.api_library_dir.empty());
}

TEST(KinovaComm, ConstructorThrowsWhenLibrariesMissing) {
  kinova::KinovaCommConfig config;
  config.robot_type = "j2n6s200";
  config.api_library_dir = "/nonexistent_dir";
  EXPECT_THROW(kinova::KinovaComm comm(config), kinova::KinovaCommException);
  config.connection_type = "Ethernet";
  EXPECT_THROW(kinova::KinovaComm comm(config), kinova::KinovaCommException);
}
