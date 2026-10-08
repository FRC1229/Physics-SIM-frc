#pragma once

// ================================================================
// Swerve drivetrain physics for the simulator (any 4-module swerve).
// Reads the voltages the robot code sends to the drive and steer motors,
// runs motor physics, and writes back what the real sensors would read
// (motor encoders, steering encoders, gyro). Also tracks where the robot
// TRULY is, so it can be compared against the robot code's estimated pose.
// ================================================================

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "sim/FieldGeometry.h"
#include "sim/hardware/SimHardware.h"

namespace robotsim {

// One swerve module's hardware, connected by the robot binding (sim/robot/).
struct SwerveModuleHardware {
  std::unique_ptr<SimMotor> drive;
  std::unique_ptr<SimMotor> steer;
  std::unique_ptr<SimAbsoluteEncoder> steerEncoder;  // optional (module angle sensor)
};

// The whole drivetrain: modules in the SAME order as `kinematics`.
struct DrivetrainHardware {
  std::array<SwerveModuleHardware, 4> modules;
  std::unique_ptr<SimGyro> gyro;
  frc::SwerveDriveKinematics<4> kinematics;
};

class SwerveModuleSim {
 public:
  explicit SwerveModuleSim(SwerveModuleHardware hardware);

  // Steps the physics and updates the module's sensors. Returns the module's
  // true state: wheel ground speed and steering angle.
  frc::SwerveModuleState Update(units::second_t dt, units::volt_t battery);
  // Gravity on a ramp speeds the wheel up or slows it down (m/s^2 along its rolling direction)
  void Accelerate(double metersPerSecondSquared, units::second_t dt);
  units::ampere_t CurrentDraw() const;

 private:
  SwerveModuleHardware m_hw;
  frc::sim::DCMotorSim m_drivePhysics;  // output = wheel
  frc::sim::DCMotorSim m_steerPhysics;  // output = module angle
  double m_driveMotorRpm = 0.0;
  double m_steerMotorRpm = 0.0;
};

class DrivetrainSim {
 public:
  explicit DrivetrainSim(DrivetrainHardware hardware);

  void Update(units::second_t dt, units::volt_t battery);

  // "Picks up" the robot and puts it here. The gyro turns with it, like a real robot.
  void ResetPose(const frc::Pose2d& pose);

  // Which field structures the robot bumps into (bumps / trenches can be passable).
  void SetFieldOptions(const FieldOptions& options) { m_obstacles = RobotObstacles(options); }

  frc::Pose2d GetPose() const { return m_pose; }
  // Same, plus how high the robot is and how it tilts (driving over a bump)
  frc::Pose3d GetPose3d() const;
  frc::ChassisSpeeds GetRobotRelativeSpeeds() const { return m_robotSpeeds; }
  frc::ChassisSpeeds GetFieldRelativeSpeeds() const;
  units::ampere_t CurrentDraw() const;

  // Bumping into walls / structures: how many times, and for how long in total
  int Collisions() const { return m_collisions; }
  double ContactSeconds() const { return m_contactSeconds; }
  bool InContact() const { return m_inContact; }
  void ResetCollisions() {
    m_collisions = 0;
    m_contactSeconds = 0.0;
    m_firstContact.reset();
  }
  // Where the robot was the first time it hit something since ResetCollisions()
  std::optional<frc::Pose2d> FirstContact() const { return m_firstContact; }

 private:
  // Ramps: gravity along each wheel, and less ground covered on a slope
  frc::SwerveModuleState ApplySlope(size_t module, frc::SwerveModuleState state, units::second_t dt);
  void UpdateTilt();

  frc::SwerveDriveKinematics<4> m_kinematics;
  std::array<std::unique_ptr<SwerveModuleSim>, 4> m_modules;
  std::unique_ptr<SimGyro> m_gyro;
  frc::Pose2d m_pose;
  frc::ChassisSpeeds m_robotSpeeds;
  std::vector<Box> m_obstacles;
  double m_batteryVolts = 12.0;
  int m_collisions = 0;
  double m_contactSeconds = 0.0;
  bool m_inContact = false;
  std::optional<frc::Pose2d> m_firstContact;
  double m_heightMeters = 0.0;
  double m_pitchRadians = 0.0;  // + = nose down
  double m_rollRadians = 0.0;   // + = left side up
};

}  // namespace robotsim
