#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "kinova_msgs/srv/home_arm.hpp"
#include "kinova_msgs/srv/start.hpp"
#include "kinova_msgs/srv/stop.hpp"
#include "kinova_system_test_fixture.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class KinovaSystemServicesTest : public KinovaSystemTest
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void configureWithServices()
  {
    ASSERT_NO_FATAL_FAILURE(configure(kinova::kFakeHomeDeg, "<param name=\"enable_services\">true</param>"));
  }

  std::string home()
  {
    auto response = std::make_shared<kinova_msgs::srv::HomeArm::Response>();
    sys.handleHomeArm(std::make_shared<kinova_msgs::srv::HomeArm::Request>(), response);
    return response->homearm_result;
  }
  std::string stop()
  {
    auto response = std::make_shared<kinova_msgs::srv::Stop::Response>();
    sys.handleStop(std::make_shared<kinova_msgs::srv::Stop::Request>(), response);
    return response->stop_result;
  }
  std::string start()
  {
    auto response = std::make_shared<kinova_msgs::srv::Start::Response>();
    sys.handleStart(std::make_shared<kinova_msgs::srv::Start::Request>(), response);
    return response->start_result;
  }
};

}  // namespace

TEST_F(KinovaSystemServicesTest, HomeRejectedWhileArmClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());
  claim(armVelocities());

  EXPECT_EQ(home(), "Rejected: deactivate arm_controller before homing");
  EXPECT_EQ(sys.fake->homeCount(), 0);
}

TEST_F(KinovaSystemServicesTest, HomeRejectedWhileStopped)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());
  EXPECT_EQ(stop(), "Arm stopped");

  EXPECT_EQ(home(), "Rejected: arm stopped (call in/start first)");
  EXPECT_EQ(sys.fake->homeCount(), 0);
}

TEST_F(KinovaSystemServicesTest, StartRejectedWhileArmClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());
  claim(armVelocities());
  EXPECT_EQ(stop(), "Arm stopped");

  EXPECT_EQ(start(), "Rejected: deactivate arm_controller before starting");
  EXPECT_TRUE(sys.fake->isStopped());
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->rawCalls(), 0);
}

TEST_F(KinovaSystemServicesTest, HomeAcceptedWhenIdle)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());

  EXPECT_EQ(home(), "KINOVA ARM HAS BEEN RETURNED HOME");
  EXPECT_EQ(sys.fake->homeCount(), 1);
}

TEST_F(KinovaSystemServicesTest, WriteSilentAfterStopService)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());
  claim(armVelocities());
  setCommand(armVelocity(1), 0.1);

  EXPECT_EQ(stop(), "Arm stopped");
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 0);

  // in/start is only accepted with the arm released; reclaiming resumes streaming.
  release(armVelocities());
  EXPECT_EQ(start(), "Arm started");
  claim(armVelocities());
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 1);
  EXPECT_GT(sys.fake->lastVelocityCommand().Actuator1, 0.0f);
}

TEST_F(KinovaSystemServicesTest, HandlersToleratePreConfigure)
{
  // Not configured: device_ is null.
  EXPECT_EQ(home().rfind("Rejected: hardware not configured", 0), 0u);
  EXPECT_EQ(stop().rfind("Rejected: hardware not configured", 0), 0u);
  EXPECT_EQ(start().rfind("Rejected: hardware not configured", 0), 0u);
}

TEST_F(KinovaSystemServicesTest, ServicesAdvertised)
{
  ASSERT_NO_FATAL_FAILURE(configureWithServices());
  auto node = std::make_shared<rclcpp::Node>("services_probe");
  const std::string base = "/j2n6s200_driver/in/";
  bool found = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!found && std::chrono::steady_clock::now() < deadline) {
    const auto services = node->get_service_names_and_types();
    found = services.count(base + "home_arm") && services.count(base + "start") &&
      services.count(base + "stop");
    if (!found) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  EXPECT_TRUE(found);

  // Teardown is idempotent and removes the services.
  EXPECT_EQ(sys.on_cleanup(lifecycle_state), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.on_shutdown(lifecycle_state), CallbackReturn::SUCCESS);
}
