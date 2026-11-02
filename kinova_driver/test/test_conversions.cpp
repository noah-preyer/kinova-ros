#include <gtest/gtest.h>

#include <cmath>

#include "kinova_driver/conversions.hpp"

using namespace kinova_driver;

TEST(Conversions, VelocityHandlesTwoComplementEncoding) {
  EXPECT_NEAR(kinovaVelocityToRad(10.0), 0.174533, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(350.0), -0.174533, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(-5.0), -0.087266, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(180.0), M_PI, 1e-9);
}

TEST(Conversions, FingerTurns) {
  EXPECT_NEAR(fingerTurnsToRad(6800.0), 1.396263, 1e-6);
  EXPECT_NEAR(fingerRadToTurns(0.698132), 3400.0, 1e-2);
  EXPECT_NEAR(fingerRadToTurns(fingerTurnsToRad(1234.0)), 1234.0, 1e-9);
}

TEST(Conversions, UnwrapNear) {
  EXPECT_NEAR(unwrapNear(0.1, 2 * M_PI - 0.1), 2 * M_PI + 0.1, 1e-12);
  EXPECT_NEAR(unwrapNear(6.2, 0.05), 6.2 - 2 * M_PI, 1e-12);
  EXPECT_NEAR(unwrapNear(1.0, 1.1), 1.0, 1e-12);
}
