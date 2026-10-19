#include <cstdio>
#include <cstdlib>

#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  // Exit without running global destructors. PathPlanner's AutoBuilder keeps a
  // PIDController in a global; destroying it after WPILib's SendableRegistry is
  // gone crashes the process (Linux), which fails the test run even though
  // every test passed.
  std::fflush(nullptr);
  std::_Exit(ret);
}
