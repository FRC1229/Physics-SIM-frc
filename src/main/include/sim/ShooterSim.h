#pragma once

// ================================================================
// Shooter, turret and feeder physics for the simulator (any robot with a
// flywheel shooter on a turret). Flywheel and turret run real motor physics
// from the voltages the robot code sends. When the feeders run, balls launch with the flywheel's
// speed, the turret's direction AND the robot's own velocity, then fly
// (gravity + drag) and are scored or missed against the real hub.
//
// Results on the dashboard under Sim/Shots/...
// ================================================================

#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <networktables/StructArrayTopic.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "sim/Ballistics.h"
#include "sim/FieldBalls.h"
#include "sim/SimConstants.h"
#include "sim/hardware/SimHardware.h"

namespace robotsim {

struct ShotStats {
  int fired = 0;
  int scored = 0;
  int missedLong = 0;
  int missedShort = 0;
  int hitHubSide = 0;
  int hitFloor = 0;
  double sumAbsLongShort = 0.0;
  double sumAbsLeftRight = 0.0;
  int finished = 0;
  ShotResult last;
};

// One shot, for tuning: what the robot was doing when the ball left, and what happened.
struct ShotRecord {
  double launchTimeSeconds = 0.0;
  frc::Pose2d robotPose;           // TRUE pose at release
  frc::ChassisSpeeds fieldSpeeds;  // TRUE robot velocity at release (field frame)
  double flywheelRpm = 0.0;
  double targetRpm = 0.0;          // what the robot code was asking for
  double turretRobotDeg = 0.0;     // TRUE turret angle, robot-relative (0 = forward)
  double distanceToHub = 0.0;      // turret -> real hub center at release, meters
  bool finished = false;
  ShotResult result;
};

// The robot's shooter hardware, connected by the robot binding (sim/robot/).
struct ShooterHardware {
  std::unique_ptr<SimMotor> flywheel;               // the motor whose encoder the code reads
  std::unique_ptr<SimMotor> turret;
  std::vector<std::unique_ptr<SimMotor>> feeders;   // a ball moves only while ALL of these run
  frc::Translation2d turretOffset;                  // turret center from robot center (robot frame)
  std::function<bool()> isRed;                      // which hub the robot is shooting at
  std::function<double()> targetRpm;                // optional: the code's flywheel target (shot records)
};

class ShooterSim {
 public:
  // Finished shots (missed or scored) are put back on `field`.
  ShooterSim(ShooterHardware hardware, FieldBalls& field);

  void Update(units::second_t dt, units::volt_t battery, const frc::Pose2d& truePose,
              const frc::ChassisSpeeds& fieldSpeeds);

  void ResetStats() { m_stats = ShotStats{}; }
  void Refill() { m_ballsInRobot = constants::kHopperCapacity; }
  void AddBalls(int count) { m_ballsInRobot = std::min(m_ballsInRobot + count, constants::kHopperCapacity); }
  int BallsInRobot() const { return m_ballsInRobot; }
  void SetBallsInRobot(int count) { m_ballsInRobot = std::clamp(count, 0, constants::kHopperCapacity); }
  const ShotStats& Stats() const { return m_stats; }
  // Fed but not out of the shooter yet, or still in the air
  bool BallsInFlight() const { return !m_balls.empty() || !m_ballsInTransit.empty(); }
  const std::vector<ShotRecord>& Records() const { return m_records; }
  void ClearRecords() { m_records.clear(); }
  frc::Pose3d TurretComponent() const;  // robot-relative pose for AdvantageScope
  units::ampere_t CurrentDraw() const;

 private:
  struct FlyingBall {
    Ball ball;
    Hub hub;
    std::vector<frc::Pose3d> path;
    size_t recordIndex = 0;  // into m_records
  };

  void UpdateFlywheel(units::second_t dt, double vbus);
  double FlywheelFrictionVolts(double appliedVolts) const;
  void UpdateTurret(units::second_t dt, double vbus);
  bool IsFeeding(double dtSeconds, double vbus);
  void Launch(const frc::Pose2d& truePose, const frc::ChassisSpeeds& fieldSpeeds);
  void UpdateBalls(double dtSeconds);
  void Record(const ShotResult& result, const std::vector<frc::Pose3d>& path);
  void PublishTelemetry();
  double FlywheelRpm() const;
  double TurretDegrees() const;  // robot-relative, 0 = forward, + = counter-clockwise

  ShooterHardware m_hw;
  FieldBalls& m_field;

  frc::sim::FlywheelSim m_flywheelPhysics;
  frc::sim::DCMotorSim m_turretPhysics;  // output = turret angle

  double m_feedProgress = 0.0;
  double m_clockSeconds = 0.0;
  std::deque<double> m_ballsInTransit;  // times the fed balls leave the shooter
  int m_ballsInRobot = constants::kStartingBalls;
  std::vector<FlyingBall> m_balls;
  ShotStats m_stats;
  std::vector<ShotRecord> m_records;

  nt::StructArrayPublisher<frc::Pose3d> m_ballPublisher;
  nt::StructArrayPublisher<frc::Pose3d> m_lastPathPublisher;
  nt::StructArrayPublisher<frc::Pose3d> m_currentPathPublisher;
};

}  // namespace robotsim
