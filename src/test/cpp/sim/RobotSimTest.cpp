// ================================================================
// End-to-end tests: the REAL robot code (RobotContainer, subsystems,
// commands) running against the simulator, in real time.
// ================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>

#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>

#include "RobotContainer.h"
#include "sim/FieldBalls.h"
#include "sim/RobotSim.h"
#include "sim/SimConstants.h"
#include "SimTestHarness.h"

using namespace robotsim;

class RobotSimTest : public simtest::SimFixture {};

TEST_F(RobotSimTest, DrivesForward) {
  const frc::Pose2d start = s_sim->GetTruePose();
  DriveFor(1.0, 0.0, 0.0, 1.5);
  const frc::Pose2d end = s_sim->GetTruePose();

  EXPECT_GT((end.X() - start.X()).value(), 1.0) << "robot did not drive forward";
  EXPECT_LT((end.X() - start.X()).value(), 2.2);
  EXPECT_NEAR(end.Y().value(), start.Y().value(), 0.15);
  EXPECT_NEAR((end.Rotation() - start.Rotation()).Degrees().value(), 0.0, 5.0);
}

TEST_F(RobotSimTest, StrafesLeft) {
  const frc::Pose2d start = s_sim->GetTruePose();
  DriveFor(0.0, 1.0, 0.0, 1.5);
  const frc::Pose2d end = s_sim->GetTruePose();

  EXPECT_GT((end.Y() - start.Y()).value(), 0.8) << "modules did not steer to 90 deg";
  EXPECT_NEAR(end.X().value(), start.X().value(), 0.25);
}

TEST_F(RobotSimTest, TurnsCounterClockwise) {
  const frc::Pose2d start = s_sim->GetTruePose();
  DriveFor(0.0, 0.0, 1.0, 1.5);
  const double turnedDeg = (s_sim->GetTruePose().Rotation() - start.Rotation()).Degrees().value();
  EXPECT_GT(turnedDeg, 40.0);
  EXPECT_LT(turnedDeg, 110.0);
}

TEST_F(RobotSimTest, OdometryFollowsTheRobotWhileDriving) {
  DriveFor(1.0, 0.5, 0.0, 1.5);
  const double error = s_sim->GetTruePose().Translation().Distance(
      s_robot->m_drive.GetEstimatedPose().Translation()).value();
  EXPECT_LT(error, 0.2);
}

TEST_F(RobotSimTest, LimelightsPullBadOdometryBackToTheTruth) {
  const frc::Pose2d truth = s_sim->GetTruePose();
  s_robot->m_drive.ResetOdometry(
      frc::Pose2d{truth.X() + 0.6_m, truth.Y() - 0.6_m, truth.Rotation()});
  RunFor(3.0);
  const double error = truth.Translation().Distance(s_robot->m_drive.GetEstimatedPose().Translation()).value();
  EXPECT_LT(error, 0.15) << "vision did not correct the estimated pose";
}

TEST_F(RobotSimTest, ShootCommandFiresBallsAtTheHub) {
  frc::SmartDashboard::PutNumber("Shooter/TurretAngleOffsetDeg", 0.0);
  // ~2.9 m straight in front of the blue hub (closer than 2.1 m the code refuses to shoot)
  PlaceRobot(frc::Pose2d{1.6_m, constants::kBlueHubCenter.Y(), frc::Rotation2d{}});
  frc2::CommandPtr shoot = s_sim->ShootCommand();
  frc2::CommandScheduler::GetInstance().Schedule(shoot);
  RunFor(5.0);
  std::cout << "[ShootCommand from 2.9 m] target RPM "
            << frc::SmartDashboard::GetNumber("Shooter/TargetRPM", 0) << ", flywheel RPM "
            << frc::SmartDashboard::GetNumber("Sim/Flywheel/RPM", 0) << ", aim distance "
            << frc::SmartDashboard::GetNumber("SOTM Distance", 0) << " m, turret setpoint "
            << frc::SmartDashboard::GetNumber("SOTM TurretDeg", 0) << ", turret actual "
            << frc::SmartDashboard::GetNumber("Sim/Turret/RobotRelativeDeg", 0) + 180.0 << "\n";
  shoot.Cancel();
  RunFor(2.0);  // let the last balls land

  const double fired = frc::SmartDashboard::GetNumber("Sim/Shots/Fired", 0);
  const double scored = frc::SmartDashboard::GetNumber("Sim/Shots/Scored", 0);
  std::cout << "[ShootCommand from 2.9 m] fired " << fired << ", scored " << scored
            << ", last: " << frc::SmartDashboard::GetString("Sim/Shots/LastResult", "?")
            << " (long/short " << frc::SmartDashboard::GetNumber("Sim/Shots/LastLongShortM", 0)
            << " m, left/right " << frc::SmartDashboard::GetNumber("Sim/Shots/LastLeftRightM", 0) << " m)\n";
  EXPECT_GE(fired, 3) << "flywheel never reached speed or the indexer never fed";
  EXPECT_GT(scored, 0) << "no shot went in from 2.9 m straight in front of the hub";
}

TEST_F(RobotSimTest, IntakeLowersAndPicksUpBallsInFrontOfTheBumper) {
  // Three balls in front of the bumper, nothing else on the field
  const frc::Pose2d robot = s_sim->GetTruePose();
  FieldBalls& field = s_sim->GetFieldBalls();
  field.Clear();
  // Side by side (a ball is 0.15 m wide, so they don't overlap) just past the intake-side bumper
  const units::meter_t out{constants::kIntakeAtBack ? -0.55 : 0.55};
  for (double left : {-0.17, 0.0, 0.17}) {
    field.Add(robot.TransformBy(frc::Transform2d{out, units::meter_t{left}, frc::Rotation2d{}}).Translation());
  }
  const int before = s_sim->BallsInRobot();
  const int expected = std::min(3, constants::kHopperCapacity - before);

  frc2::CommandPtr intake = s_sim->IntakeCommand();
  frc2::CommandScheduler::GetInstance().Schedule(intake);
  RunFor(3.0);
  const double armDeg = s_robot->m_intake.GetPosition();
  intake.Cancel();
  RunFor(0.1);

  std::cout << "[Intake] arm at " << armDeg << " deg (code reading), picked up "
            << s_sim->BallsInRobot() - before << " of 3\n";
  EXPECT_NEAR(armDeg, IntakeSubsystem::kLowered, IntakeSubsystem::kTolerance + 3.0)
      << "IntakeSubsystem::GoToTarget did not drive the simulated arm down";
  EXPECT_EQ(s_sim->BallsInRobot() - before, expected);
  EXPECT_EQ(field.Count(), 3 - expected);
}

// Exactly what the driver does: A (button 1) toggles the arm down, hold the left
// trigger (axis 2) for the rollers, then back into the neutral-zone pile.
TEST_F(RobotSimTest, DriverControlsIntakeBallsFromThePile) {
  s_sim->GetFieldBalls().Reset(StartingFieldBalls());
  s_sim->ResetMechanisms();
  RunOneLoop();
  EXPECT_NEAR(s_robot->m_intake.GetPosition(), 0.0, 0.5) << "resetting the sim's arm must reset what the code reads";
  s_robot->m_intake.SetTarget(IntakeSubsystem::kRaised);
  s_sim->Shooter().SetBallsInRobot(0);
  // Intake side facing the pile
  const double pileNearX = constants::kFieldLengthMeters / 2.0 -
                           (constants::kNeutralBallColumns - 1) * constants::kBallGridSpacingMeters / 2.0;
  const double toPile = constants::kIntakeAtBack ? 180.0 : 0.0;
  PlaceRobot(frc::Pose2d{units::meter_t{pileNearX - 1.0}, units::meter_t{constants::kFieldWidthMeters / 2.0},
                         frc::Rotation2d{units::degree_t{toPile}}});

  SetButton(kDriverPort, 1, true);
  RunFor(0.1);
  SetButton(kDriverPort, 1, false);
  RunFor(2.0);  // arm comes down
  const double armDeg = s_robot->m_intake.GetPosition();
  EXPECT_NEAR(frc::SmartDashboard::GetNumber("Sim/Intake/ArmDeg", -1), armDeg, 1.0)
      << "the code's arm reading drifted from the simulated arm";
  frc::sim::DriverStationSim::SetJoystickAxis(kDriverPort, 2, 1.0);
  frc::sim::DriverStationSim::NotifyNewData();
  const double intoPile = constants::kIntakeAtBack ? -1.0 : 1.0;
  DriveFor(intoPile * 1.0, 0.0, 0.0, 2.0);
  frc::sim::DriverStationSim::SetJoystickAxis(kDriverPort, 2, 0.0);
  frc::sim::DriverStationSim::NotifyNewData();

  const frc::Pose2d end = s_sim->GetTruePose();
  double nearestFront = 99.0;
  for (const frc::Translation2d& ball : s_sim->GetFieldBalls().Positions()) {
    const frc::Translation2d local = (ball - end.Translation()).RotateBy(-end.Rotation());
    if (std::abs(local.Y().value()) < 0.3 && std::abs(local.X().value()) < std::abs(nearestFront)) {
      nearestFront = local.X().value();
    }
  }
  std::cout << "[Driver intake] arm " << armDeg << " deg, roller output "
            << s_robot->m_roller.m_intakeRoller.GetAppliedOutput() << ", picked up " << s_sim->BallsInRobot()
            << ", robot ended at x " << end.X().value() << " heading " << end.Rotation().Degrees().value()
            << ", nearest ball in line " << nearestFront << " m (robot frame)\n";
  EXPECT_GT(armDeg, 150.0) << "button 1 did not lower the arm";
  EXPECT_GE(s_sim->BallsInRobot(), 5) << "backing into the pile with the trigger held picked up too few";
}

// Triangle on a PS4 (through DS4Windows as an Xbox controller) = Y = button 4:
// the robot code's ZeroHeading. Forward on the stick must then be "away from the driver".
TEST_F(RobotSimTest, DriverTriangleMakesTheRobotsFacingForward) {
  // Robot turned to face +y, then the driver presses Triangle
  const frc::Pose2d start{constants::kStartPose.Translation(), frc::Rotation2d{90_deg}};
  PlaceRobot(start);
  SetButton(kDriverPort, 4, true);
  RunFor(0.1);
  SetButton(kDriverPort, 4, false);
  RunFor(0.2);

  // Left stick forward (the code drives with -axis 1)
  frc::sim::DriverStationSim::SetJoystickAxis(kDriverPort, 1, -0.5);
  frc::sim::DriverStationSim::NotifyNewData();
  RunFor(1.0);
  frc::sim::DriverStationSim::SetJoystickAxis(kDriverPort, 1, 0.0);
  frc::sim::DriverStationSim::NotifyNewData();
  RunFor(0.5);

  const frc::Pose2d end = s_sim->GetTruePose();
  const double estimatedHeading = s_robot->m_drive.GetEstimatedPose().Rotation().Degrees().value();
  std::cout << "[Triangle] moved dx " << (end.X() - start.X()).value() << " m, dy " << (end.Y() - start.Y()).value()
            << " m; code's heading " << estimatedHeading << " deg (true " << end.Rotation().Degrees().value()
            << ")\n";
  EXPECT_GT((end.Y() - start.Y()).value(), 0.3) << "stick forward should drive the way the robot faced";
  EXPECT_LT(std::abs((end.X() - start.X()).value()), 0.15);
  EXPECT_NEAR(estimatedHeading, end.Rotation().Degrees().value(), 3.0)
      << "pressing Triangle must not throw off the code's field heading";
}

// The robot code drives straight over the bump below the blue hub: the robot
// climbs, tips nose-up then nose-down, and comes out the other side.
TEST_F(RobotSimTest, RobotDrivesOverTheBumpAndTipsOnTheRamps) {
  const double hubX = constants::kBlueHubCenter.X().value();
  const double bumpY = constants::kBlueHubCenter.Y().value() - constants::kHubHalfSizeMeters -
                       constants::kBumpLengthMeters / 2.0;
  const frc::Pose2d start{units::meter_t{hubX - 1.5}, units::meter_t{bumpY}, frc::Rotation2d{}};
  PlaceRobot(start);

  frc2::CommandPtr drive = DriveCommand(1.5, 0.0, 0.0);
  frc2::CommandScheduler::GetInstance().Schedule(drive);
  double maxHeight = 0.0;
  double noseUp = 0.0;
  double noseDown = 0.0;
  for (int i = 0; i < 125; ++i) {  // 2.5 s
    RunOneLoop();
    const frc::Pose3d pose = s_sim->Drivetrain().GetPose3d();
    maxHeight = std::max(maxHeight, pose.Z().value());
    const double pitchDeg = units::degree_t{pose.Rotation().Y()}.value();
    noseUp = std::min(noseUp, pitchDeg);  // WPILib: - pitch = nose up
    noseDown = std::max(noseDown, pitchDeg);
  }
  drive.Cancel();
  DriveFor(0.0, 0.0, 0.0, 0.3);

  const frc::Pose2d end = s_sim->GetTruePose();
  const double odometryError = end.Translation().Distance(s_robot->m_drive.GetEstimatedPose().Translation()).value();
  std::cout << "[Bump] max height " << maxHeight << " m, nose up " << -noseUp << " deg, nose down " << noseDown
            << " deg, ended at x " << end.X().value() << " (bump ends at " << hubX + constants::kBumpHalfDepthMeters
            << "), odometry error " << odometryError << " m\n";
  // The bump's flat top is shorter than the wheelbase, so the wheels never all sit
  // on top at once: the middle of the robot peaks around 60% of the bump height
  EXPECT_GT(maxHeight, 0.5 * constants::kBumpHeightMeters) << "robot never got up onto the bump";
  EXPECT_GT(-noseUp, 5.0) << "robot should tip nose-up climbing the ramp";
  EXPECT_GT(noseDown, 5.0) << "and nose-down coming off it";
  EXPECT_GT(end.X().value(), hubX + constants::kBumpHalfDepthMeters + constants::kBumperLengthMeters / 2.0)
      << "robot should have made it over";
  EXPECT_NEAR(s_sim->Drivetrain().GetPose3d().Z().value(), 0.0, 1e-9) << "back on the carpet";
}

TEST_F(RobotSimTest, RobotDrivingIntoTheHubStopsAtItsFace) {
  const double hubFrontX = constants::kBlueHubCenter.X().value() - constants::kHubHalfSizeMeters;
  const frc::Pose2d start{units::meter_t{hubFrontX - 1.2}, constants::kBlueHubCenter.Y(), frc::Rotation2d{}};
  PlaceRobot(start);
  DriveFor(1.5, 0.0, 0.0, 2.0);  // would travel ~3 m without the hub in the way

  const double frontBumperX = s_sim->GetTruePose().X().value() + constants::kBumperLengthMeters / 2.0;
  EXPECT_LE(frontBumperX, hubFrontX + 1e-3) << "robot drove into the hub";
  EXPECT_GT(frontBumperX, hubFrontX - 0.05) << "robot should be pressed against the hub";
}

// Holds the copilot's shoot button (LB = button 5), exactly like a driver would.
// ShooterCommand works out the turret angle and AutoLock must turn the turret there.
TEST_F(RobotSimTest, CopilotShootButtonLocksTheTurretOntoTheHub) {
  // Robot turned 60 deg left, so the hub is well off to its right
  const frc::Pose2d start{constants::kStartPose.Translation(), frc::Rotation2d{60_deg}};
  PlaceRobot(start);

  constexpr int kCopilotPort = 1;
  constexpr int kShootButton = 5;
  frc::sim::DriverStationSim::SetJoystickButtonCount(kCopilotPort, 10);
  frc::sim::DriverStationSim::SetJoystickButton(kCopilotPort, kShootButton, true);
  frc::sim::DriverStationSim::NotifyNewData();
  RunFor(3.0);
  const double setpoint = frc::SmartDashboard::GetNumber("SOTM TurretDeg", -1.0);
  const double turret = s_robot->m_turret.readEncoder();
  frc::sim::DriverStationSim::SetJoystickButton(kCopilotPort, kShootButton, false);
  frc::sim::DriverStationSim::NotifyNewData();
  RunFor(0.1);

  std::cout << "[AutoLock] turret setpoint " << setpoint << " deg, turret at " << turret << " deg\n";
  ASSERT_GE(setpoint, 0.0) << "ShooterCommand never ran (button binding?)";
  EXPECT_LT(std::abs(s_robot->m_turret.shortestAngleError(setpoint, turret)), 3.0)
      << "turret did not lock onto the angle ShooterCommand asked for";
  EXPECT_GT(std::abs(s_robot->m_turret.shortestAngleError(180.0, turret)), 20.0)
      << "turret never moved from straight ahead";
}

// Smoothness: with the whole ball pile on the field and a burst of shots in
// the air, every sim loop must stay far below the 20 ms robot loop.
TEST_F(RobotSimTest, SimLoopStaysFastWhileShooting) {
  s_sim->GetFieldBalls().Reset(StartingFieldBalls());
  PlaceRobot(frc::Pose2d{1.6_m, constants::kBlueHubCenter.Y(), frc::Rotation2d{}});
  s_sim->Shooter().SetBallsInRobot(8);
  frc2::CommandPtr shoot = s_sim->ShootCommand();
  frc2::CommandScheduler::GetInstance().Schedule(shoot);
  double worstMs = 0.0;
  double totalMs = 0.0;
  int loops = 0;
  for (; loops < 200; ++loops) {
    RunOneLoop();
    const double ms = frc::SmartDashboard::GetNumber("Sim/LoopMs", 0.0);
    worstMs = std::max(worstMs, ms);
    totalMs += ms;
  }
  shoot.Cancel();
  std::cout << "[Smoothness] sim loop average " << totalMs / loops << " ms, worst " << worstMs << " ms\n";
  EXPECT_LT(worstMs, 10.0) << "the sim would stutter (it has to fit inside a 20 ms loop)";
}
