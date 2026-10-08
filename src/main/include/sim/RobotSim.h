#pragma once

// ================================================================
// The whole simulated robot: drivetrain + Limelights + intake + shooter,
// plus the balls lying on the field.
// Created automatically once the robot code starts in simulation
// (see SimHook.cpp), then updated every 20 ms robot loop.
//
// Dashboard test buttons (SmartDashboard -> Sim/Commands/...):
//   Shoot (aim + spin + fire)      - your ShooterCommand + AutoLock + LaunchCommand
//   Shoot While Strafing           - same, while driving side to side
//   Shoot While Driving At Hub     - same, while driving toward/away from the hub
//   Intake (lower + rollers)       - lowers the intake and runs RunIntake
//   Move Robot To Shot Test Spot   - Sim/ShotTest/DistanceM and AngleDeg from your hub
//   Shot Sweep                     - your shoot commands from 1.5 - 4 m, hit % per distance
//   Reset Robot To Start, Snap Odometry To True Pose, Reset Shot Stats, Refill Balls,
//   Reset Field Balls
//
// Field switches: Sim/Field/RobotCanCrossBumps, Sim/Field/RobotFitsUnderTrench
// ================================================================

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <units/time.h>

#include <frc2/command/Subsystem.h>
#include "sim/DrivetrainSim.h"
#include "sim/FieldBalls.h"
#include "sim/FieldGeometry.h"
#include "sim/IntakeSim.h"
#include "sim/LimelightSim.h"
#include "sim/ShooterSim.h"

namespace robotsim {

// The robot's hardware for each mechanism sim (made by the robot binding).
struct RobotHardware {
  DrivetrainHardware drivetrain;
  std::unique_ptr<SimMotor> intakeArm;
  std::unique_ptr<SimMotor> intakeRoller;
  ShooterHardware shooter;
};

// What the sim needs from the robot CODE (not its hardware): where it thinks
// the robot is, a way to drive it for test buttons, and its own commands for
// the shoot / intake test buttons. Made by the robot binding.
struct RobotCodeHooks {
  std::function<frc::Pose2d()> estimatedPose;
  std::function<void(const frc::Pose2d&)> resetOdometry;
  std::function<void(const frc::ChassisSpeeds&)> driveRobotRelative;
  std::function<void()> stopDriving;
  frc2::Subsystem* driveSubsystem = nullptr;  // so test drives take over from the joystick
  std::function<bool()> isRed;
  std::function<frc2::CommandPtr()> shootCommand;   // aim + spin up + fire
  std::function<frc2::CommandPtr()> intakeCommand;  // lower the intake + run the rollers

  // Autos
  std::function<std::string()> selectedAuto;                         // picked on the dashboard chooser
  std::function<frc::Pose2d(const std::string&)> autoStartPose;      // where that auto starts on the field
  std::function<frc2::CommandPtr(const std::string&)> autoCommand;   // what the robot runs in autonomous
};

class RobotSim {
 public:
  RobotSim(RobotHardware hardware, RobotCodeHooks hooks);

  void Update(units::second_t dt);

  // Where the robot really is (not where the robot code thinks it is)
  frc::Pose2d GetTruePose() const { return m_drivetrain.GetPose(); }
  frc::ChassisSpeeds GetFieldRelativeSpeeds() const { return m_drivetrain.GetFieldRelativeSpeeds(); }
  void ResetTruePose(const frc::Pose2d& pose) { m_drivetrain.ResetPose(pose); }

  FieldBalls& GetFieldBalls() { return m_fieldBalls; }
  ShooterSim& Shooter() { return m_shooter; }
  DrivetrainSim& Drivetrain() { return m_drivetrain; }
  int BallsInRobot() const { return m_shooter.BallsInRobot(); }

  // Aim + spin up + fire, using the robot's real commands
  frc2::CommandPtr ShootCommand();

  // Autos: where an auto starts, and the command the robot runs for it
  frc::Pose2d AutoStartPose(const std::string& name) const { return m_hooks.autoStartPose(name); }
  frc2::CommandPtr AutoCommand(const std::string& name) const { return m_hooks.autoCommand(name); }
  int BallsCollected() const { return m_ballsCollected; }
  // Mechanisms back to their power-on positions (between test autos)
  void ResetMechanisms() { m_intake.ResetArm(); }
  // Lower the intake and spin the rollers, using the robot's real subsystems
  frc2::CommandPtr IntakeCommand();
  // Shoots a few balls from each of several distances; results on Sim/ShotSweep/...
  frc2::CommandPtr ShotSweepCommand();

  struct SweepResult {
    double distance = 0.0;
    int fired = 0;
    int scored = 0;
    double avgLongShort = 0.0;  // average |long/short miss|, meters
  };
  const std::vector<SweepResult>& SweepResults() const { return m_sweepResults; }
  // Drive back and forth: side to side, or toward/away from the hub
  frc2::CommandPtr ShuttleCommand(bool towardHub);

 private:
  void RegisterDashboardCommands();
  void AddCommand(std::string_view name, frc2::CommandPtr command);
  void PublishTelemetry();
  void UpdateFieldOptions();
  frc::Pose2d ShotTestPose(double distance) const;
  frc2::CommandPtr PlaceRobotCommand(std::function<frc::Pose2d()> pose);
  void RecordSweepSpot(double distance);
  void PublishSweepResults();
  void UpdateAutoPlacement();
  void UpdateAutoReport();
  void PlaceAtAutoStart(const std::string& name);

  RobotCodeHooks m_hooks;
  DrivetrainSim m_drivetrain;
  LimelightSim m_limelights;
  FieldBalls m_fieldBalls;  // before m_shooter, which keeps a reference to it
  IntakeSim m_intake;
  ShooterSim m_shooter;
  std::vector<frc2::CommandPtr> m_commands;
  int m_ballsCollected = 0;
  FieldOptions m_fieldOptions{constants::kRobotCanCrossBumps, constants::kRobotFitsUnderTrench};
  ShotStats m_sweepBaseline;
  std::vector<SweepResult> m_sweepResults;

  // Pre-match placement at the selected auto's start, and the end-of-auto report
  std::string m_placedAuto;
  int m_odometryResetCountdown = 0;  // loops until the gyro has caught up after a placement
  bool m_wasAutonomousEnabled = false;
  ShotStats m_autoStartStats;
  int m_autoStartCollected = 0;
  double m_autoStartSeconds = 0.0;
  nt::StructPublisher<frc::Pose2d> m_autoStartPublisher;
  bool m_fieldBallsPublished = false;
  unsigned m_publishedFieldBallVersion = 0;
  int m_loopsSinceFieldBallPublish = 0;

  nt::StructPublisher<frc::Pose2d> m_truePosePublisher;
  nt::StructPublisher<frc::Pose3d> m_truePose3dPublisher;
  nt::StructArrayPublisher<frc::Pose3d> m_fieldBallPublisher;
  nt::StructArrayPublisher<frc::Pose3d> m_componentPublisher;
  nt::DoublePublisher m_poseErrorPublisher;
  nt::DoublePublisher m_headingErrorPublisher;
};

}  // namespace robotsim
