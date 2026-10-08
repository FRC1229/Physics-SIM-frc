#include "sim/robot/RobotBinding.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>

#include <exception>
#include <iostream>
#include <string>

#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/FlippingUtil.h>

#include "Constants.h"
#include "commands/AutoLock.h"
#include "commands/LaunchCommand.h"
#include "commands/RunIntake.h"
#include "commands/ShooterCommand.h"
#include "sim/PrivateAccess.h"
#include "sim/SimConstants.h"

namespace robotsim {

// IndexerSubsystem keeps its feeder motors private; read them without editing the robot code.
// (Tags live in this namespace so Get() matches the friend defined by PrivateMember.)
struct IndexerKickerTag {
  using type = rev::spark::SparkMax IndexerSubsystem::*;
  friend type Get(IndexerKickerTag);
};
struct IndexerSpinTag {
  using type = rev::spark::SparkMax IndexerSubsystem::*;
  friend type Get(IndexerSpinTag);
};
template struct PrivateMember<IndexerKickerTag, &IndexerSubsystem::m_kickerMotor>;
template struct PrivateMember<IndexerSpinTag, &IndexerSubsystem::m_spinMotor>;

namespace binding {

namespace {

SwerveModuleHardware ModuleHardware(SwerveModule& module) {
  SwerveModuleHardware hardware;
  hardware.drive = std::make_unique<SparkMotor>(module.m_driveMotor, frc::DCMotor::NEO());
  hardware.steer = std::make_unique<SparkMotor>(module.m_turningMotor, frc::DCMotor::NEO());
  hardware.steerEncoder = std::make_unique<CANcoderSensor>(module.m_turningEncoder);
  return hardware;
}

}  // namespace

// Same module order as DriveSubsystem::kDriveKinematics: FL, FR, RL, RR
DrivetrainHardware MakeDrivetrainHardware(RobotContainer& robot) {
  DriveSubsystem& drive = robot.m_drive;
  return DrivetrainHardware{{ModuleHardware(drive.m_frontLeft), ModuleHardware(drive.m_frontRight),
                             ModuleHardware(drive.m_rearLeft), ModuleHardware(drive.m_rearRight)},
                            std::make_unique<Pigeon2Gyro>(drive.m_gyro),
                            drive.kDriveKinematics};
}

std::unique_ptr<SimMotor> MakeIntakeArmMotor(RobotContainer& robot) {
  return std::make_unique<SparkMotor>(robot.m_intake.m_intakeMotor, frc::DCMotor::NEO());
}

std::unique_ptr<SimMotor> MakeIntakeRollerMotor(RobotContainer& robot) {
  return std::make_unique<SparkMotor>(robot.m_roller.m_intakeRoller, frc::DCMotor::NEO());
}

ShooterHardware MakeShooterHardware(RobotContainer& robot) {
  IndexerSubsystem& indexer = robot.m_indexer;
  ShooterHardware hardware;
  // Leader + follower Vortexes; the code reads the leader's encoder
  hardware.flywheel = std::make_unique<SparkMotor>(robot.m_shooter.m_leaderMotor, frc::DCMotor::NeoVortex(2));
  hardware.turret = std::make_unique<SparkMotor>(robot.m_turret.m_turretMotor, frc::DCMotor::NEO());
  // A ball moves into the shooter only while both the spin feeder and the kicker run
  hardware.feeders.push_back(std::make_unique<SparkMotor>(indexer.*Get(IndexerSpinTag{}), frc::DCMotor::NEO()));
  hardware.feeders.push_back(std::make_unique<SparkMotor>(indexer.*Get(IndexerKickerTag{}), frc::DCMotor::NEO()));
  hardware.turretOffset = frc::Translation2d{units::meter_t{ShooterConstants::kTurretOffsetX},
                                             units::meter_t{ShooterConstants::kTurretOffsetY}};
  hardware.isRed = [&robot] { return robot.m_vision.isRed; };
  hardware.targetRpm = [] { return frc::SmartDashboard::GetNumber("Shooter/TargetRPM", 0.0); };
  return hardware;
}

// Lowers the intake (IntakeSubsystem's own target control) and spins the
// rollers with the robot's RunIntake command at the driver's left-trigger speed.
frc2::CommandPtr IntakeTestCommand(RobotContainer& r) {
  constexpr double kRollerSpeed = 0.9;
  return frc2::cmd::Parallel(
             frc2::cmd::RunOnce([&r] { r.m_intake.SetTarget(IntakeSubsystem::kLowered); }, {&r.m_intake})
                 .AndThen(frc2::cmd::Run([&r] { r.m_intake.GoToTarget(); }, {&r.m_intake})),
             RunIntake(&r.m_roller, kRollerSpeed).ToPtr())
      .FinallyDo([&r](bool) { r.m_intake.MoveIntake(0.0); });
}

// The robot's real shooting commands: spin up + aim (ShooterCommand), point the
// turret (AutoLock), and feed (LaunchCommand) once the flywheel is at speed.
frc2::CommandPtr ShootTestCommand(RobotContainer& r) {
  return frc2::cmd::Parallel(
      ShooterCommand(&r.m_shooter, &r.m_drive, &r.m_turret, &r.m_vision, true).ToPtr(),
      AutoLock(&r.m_turret, &r.m_vision).ToPtr(),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([&r] { return r.m_shooter.AtSetpoint() && r.m_shooter.HasValidShot(); }),
          LaunchCommand(&r.m_indexer, &r.m_turret, &r.m_shooter).ToPtr()));
}

// Where a PathPlanner auto starts, flipped the same way the robot code flips
// its paths (DriveSubsystem::Fieldflip, the "IsRedAlliance" dashboard chooser).
frc::Pose2d AutoStartPose(RobotContainer& robot, const std::string& name) {
  try {
    const frc::Pose2d start = pathplanner::PathPlannerAuto(name).getStartingPose();
    return robot.m_drive.Fieldflip() ? pathplanner::FlippingUtil::flipFieldPose(start) : start;
  } catch (const std::exception& error) {
    std::cerr << "[Sim] Could not load auto '" << name << "': " << error.what() << std::endl;
    return constants::kStartPose;
  }
}

// Exactly what Robot::AutonomousInit runs: zero the gyro and encoders, then the auto.
frc2::CommandPtr AutoRunCommand(RobotContainer& robot, const std::string& name) {
  return frc2::cmd::Sequence(frc2::cmd::RunOnce([&robot] {
                               robot.m_drive.ZeroHeading();
                               robot.m_drive.ResetEncoders();
                             }),
                             pathplanner::PathPlannerAuto(name).ToPtr());
}

RobotCodeHooks MakeRobotCodeHooks(RobotContainer& robot) {
  RobotCodeHooks hooks;
  hooks.estimatedPose = [&robot] { return robot.m_drive.GetEstimatedPose(); };
  hooks.resetOdometry = [&robot](const frc::Pose2d& pose) { robot.m_drive.ResetOdometry(pose); };
  hooks.driveRobotRelative = [&robot](const frc::ChassisSpeeds& speeds) {
    robot.m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false);
  };
  hooks.stopDriving = [&robot] { robot.m_drive.Stop(); };
  hooks.driveSubsystem = &robot.m_drive;
  hooks.isRed = [&robot] { return robot.m_vision.isRed; };
  hooks.shootCommand = [&robot] { return ShootTestCommand(robot); };
  hooks.intakeCommand = [&robot] { return IntakeTestCommand(robot); };
  hooks.selectedAuto = [&robot] { return robot.m_chooser.GetSelected(); };
  hooks.autoStartPose = [&robot](const std::string& name) { return AutoStartPose(robot, name); };
  hooks.autoCommand = [&robot](const std::string& name) { return AutoRunCommand(robot, name); };
  return hooks;
}

std::unique_ptr<RobotSim> MakeRobotSim(RobotContainer& robot) {
  RobotHardware hardware{MakeDrivetrainHardware(robot), MakeIntakeArmMotor(robot), MakeIntakeRollerMotor(robot),
                         MakeShooterHardware(robot)};
  return std::make_unique<RobotSim>(std::move(hardware), MakeRobotCodeHooks(robot));
}

}  // namespace binding
}  // namespace robotsim
