#pragma once

// ================================================================
// Shared harness for tests that run the REAL robot code (RobotContainer,
// subsystems, commands) against the simulator, in real time.
//
// Only one RobotContainer can exist per process (it owns the simulated CAN
// devices), so every end-to-end test suite shares this one.
// ================================================================

#include <chrono>
#include <filesystem>
#include <thread>

#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>

#include "RobotContainer.h"
#include "sim/RobotSim.h"
#include "sim/SimConstants.h"
#include "sim/robot/RobotBinding.h"

namespace simtest {

class SimFixture : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    static RobotContainer* robot = nullptr;
    static robotsim::RobotSim* sim = nullptr;
    if (robot == nullptr) {
      // The robot code loads PathPlanner settings from <working dir>/src/main/deploy,
      // but tests run from inside build/, so move up to the project folder first.
      std::filesystem::path dir = std::filesystem::current_path();
      while (!std::filesystem::exists(dir / "src/main/deploy/pathplanner/settings.json") &&
             dir.has_parent_path() && dir != dir.parent_path()) {
        dir = dir.parent_path();
      }
      std::filesystem::current_path(dir);
      // Left alive on purpose (like on a real robot, they are never destroyed);
      // tearing down CAN devices mid-process is not supported.
      robot = new RobotContainer();
      sim = robotsim::binding::MakeRobotSim(*robot).release();
    }
    s_robot = robot;
    s_sim = sim;
    // Tests place the robot themselves: no automatic "stand at the selected auto's start"
    frc::SmartDashboard::PutBoolean("Sim/Auto/PlaceAtStart", false);
  }

  void SetUp() override {
    frc2::CommandScheduler::GetInstance().CancelAll();
    ReleaseAllButtons();
    PlaceRobot(robotsim::constants::kStartPose);
    SetEnabled(true);
    RunFor(0.3);  // let the swerve modules settle
  }

  void TearDown() override {
    ReleaseAllButtons();
    frc2::CommandScheduler::GetInstance().CancelAll();
    SetEnabled(false);
    RunFor(0.1);
  }

  static void SetEnabled(bool enabled) {
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  // One robot loop = scheduler + simulator, every 20 ms in real time
  // (Phoenix sensors update in real time, so the test does too).
  static void RunFor(double seconds) {
    const int loops = static_cast<int>(seconds / 0.02);
    for (int i = 0; i < loops; ++i) {
      RunOneLoop();
    }
  }

  static void RunOneLoop() {
    frc::DriverStation::RefreshData();
    frc2::CommandScheduler::GetInstance().Run();
    s_sim->Update(20_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Presses / releases a controller button, like a driver would.
  static constexpr int kDriverPort = 0;
  static constexpr int kCopilotPort = 1;
  static void SetButton(int port, int button, bool pressed) {
    frc::sim::DriverStationSim::SetJoystickButtonCount(port, 12);
    frc::sim::DriverStationSim::SetJoystickButton(port, button, pressed);
    frc::sim::DriverStationSim::NotifyNewData();
  }
  static void ReleaseAllButtons() {
    for (int port : {kDriverPort, kCopilotPort}) {
      // Plugged-in controllers with buttons and sticks (the robot code reads both)
      frc::sim::DriverStationSim::SetJoystickButtonCount(port, 12);
      frc::sim::DriverStationSim::SetJoystickAxisCount(port, 6);
      for (int button = 1; button <= 12; ++button) {
        frc::sim::DriverStationSim::SetJoystickButton(port, button, false);
      }
    }
    frc::sim::DriverStationSim::NotifyNewData();
  }

  // Robot-relative drive (what the joystick default command would send)
  static frc2::CommandPtr DriveCommand(double vx, double vy, double omega) {
    return frc2::cmd::Run(
        [=] {
          s_robot->m_drive.Drive(units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                                 units::radians_per_second_t{omega}, false);
        },
        {&s_robot->m_drive});
  }

  static void DriveFor(double vx, double vy, double omega, double seconds) {
    frc2::CommandPtr command = DriveCommand(vx, vy, omega);
    frc2::CommandScheduler::GetInstance().Schedule(command);
    RunFor(seconds);
    command.Cancel();
    frc2::CommandPtr stop = DriveCommand(0.0, 0.0, 0.0);
    frc2::CommandScheduler::GetInstance().Schedule(stop);
    RunFor(0.5);
    stop.Cancel();
  }

  // Picks the robot up, puts it here, and tells the robot code exactly where it is.
  // The simulated Pigeon reports the new heading a few loops later (CTRE runs its
  // own sim clock), so wait for it before resetting odometry; otherwise the robot
  // code keeps the OLD gyro angle and its heading is off by the turn.
  static void PlaceRobot(const frc::Pose2d& pose) {
    s_sim->ResetTruePose(pose);
    RunFor(kGyroSettleSeconds);
    s_robot->m_drive.ResetOdometry(s_sim->GetTruePose());
  }
  static constexpr double kGyroSettleSeconds = 0.1;

  static inline RobotContainer* s_robot = nullptr;
  static inline robotsim::RobotSim* s_sim = nullptr;
};

}  // namespace simtest
