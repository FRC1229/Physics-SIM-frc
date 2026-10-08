#include "sim/ShooterSim.h"
#include "sim/season/Season.h"
#include "sim/Telemetry.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string_view>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <networktables/NetworkTableInstance.h>


namespace robotsim {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kRadPerSecToRpm = 60.0 / kTwoPi;
constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr std::string_view kTransitKey = "Sim/Model/BallTransitSec";
constexpr std::string_view kLossPerShotKey = "Sim/Model/FlywheelLossPerShot";

BallPhysics MakeBallPhysics() {
  return BallPhysics{constants::kGravity, constants::kDragFactor, constants::kBallDiameterMeters / 2.0,
                     constants::kBallLandingSpeedKept};
}

frc::Pose3d ToPose3d(const Vec3& v) {
  return frc::Pose3d{units::meter_t{v.x}, units::meter_t{v.y}, units::meter_t{v.z}, frc::Rotation3d{}};
}

}  // namespace

ShooterSim::ShooterSim(ShooterHardware hardware, FieldBalls& field)
    : m_hw(std::move(hardware)),
      m_field(field),
      m_flywheelPhysics(frc::LinearSystemId::FlywheelSystem(m_hw.flywheel->Gearbox(), constants::kFlywheelInertia,
                                                            constants::kFlywheelGearing),
                        m_hw.flywheel->Gearbox()),
      m_turretPhysics(frc::LinearSystemId::DCMotorSystem(m_hw.turret->Gearbox(), constants::kTurretInertia,
                                                         constants::kTurretGearing),
                      m_hw.turret->Gearbox()) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_ballPublisher = inst.GetStructArrayTopic<frc::Pose3d>("Sim/Balls").Publish(EveryLoop());
  // The newest ball's arc so far (draws itself in AdvantageScope as it flies)
  m_currentPathPublisher = inst.GetStructArrayTopic<frc::Pose3d>("Sim/Shots/CurrentPath").Publish(EveryLoop());
  m_lastPathPublisher = inst.GetStructArrayTopic<frc::Pose3d>("Sim/Shots/LastPath").Publish();
  // Unmeasured robot numbers, adjustable to see how much the results depend on them
  frc::SmartDashboard::SetDefaultNumber(kTransitKey, constants::kBallTransitSeconds);
  frc::SmartDashboard::SetDefaultNumber(kLossPerShotKey, constants::kFlywheelSpeedLossPerShot);
}

void ShooterSim::Update(units::second_t dt, units::volt_t battery, const frc::Pose2d& truePose,
                        const frc::ChassisSpeeds& fieldSpeeds) {
  const double vbus = battery.value();
  UpdateFlywheel(dt, vbus);
  UpdateTurret(dt, vbus);

  m_clockSeconds += dt.value();
  if (IsFeeding(dt.value(), vbus) && m_ballsInRobot > 0) {
    m_feedProgress += constants::kBallsPerSecond * dt.value();
    while (m_feedProgress >= 1.0 && m_ballsInRobot > 0) {
      m_feedProgress -= 1.0;
      // The ball is in the kicker now; it leaves the shooter a little later,
      // from wherever the robot is by then.
      m_ballsInRobot--;
      m_stats.fired++;
      m_ballsInTransit.push_back(m_clockSeconds +
                                 frc::SmartDashboard::GetNumber(kTransitKey, constants::kBallTransitSeconds));
    }
  }
  // (When the feeder stops, a half-pushed ball stays where it is: progress is kept.)
  while (!m_ballsInTransit.empty() && m_ballsInTransit.front() <= m_clockSeconds) {
    m_ballsInTransit.pop_front();
    Launch(truePose, fieldSpeeds);
  }

  UpdateBalls(dt.value());
  PublishTelemetry();
}

void ShooterSim::UpdateFlywheel(units::second_t dt, double vbus) {
  const double applied = m_hw.flywheel->Update(FlywheelRpm() * constants::kFlywheelGearing, vbus, dt.value());
  m_flywheelPhysics.SetInputVoltage(units::volt_t{applied - FlywheelFrictionVolts(applied)});
  m_flywheelPhysics.Update(dt);
  const double motorRpm = FlywheelRpm() * constants::kFlywheelGearing;
  m_hw.flywheel->MoveEncoder(motorRpm / 60.0 * dt.value(), motorRpm);
}

// Voltage lost to friction: the part of the team's measured feedforward that an
// ideal motor would not need (kS plus the extra kV).
double ShooterSim::FlywheelFrictionVolts(double appliedVolts) const {
  const double motorRpm = FlywheelRpm() * constants::kFlywheelGearing;
  if (std::abs(motorRpm) < 1.0) {
    // Standing still: friction only pushes back as hard as the motor pushes
    return std::clamp(appliedVolts, -constants::kFlywheelFrictionVolts, constants::kFlywheelFrictionVolts);
  }
  const double idealVoltsPerRpm = 1.0 / (m_hw.flywheel->Gearbox().Kv.value() * kRadPerSecToRpm);
  const double extraVoltsPerRpm = constants::kFlywheelMeasuredVoltsPerRpm - idealVoltsPerRpm;
  return std::copysign(constants::kFlywheelFrictionVolts, motorRpm) + extraVoltsPerRpm * motorRpm;
}

void ShooterSim::UpdateTurret(units::second_t dt, double vbus) {
  const double degreesBefore = TurretDegrees();
  const double volts = m_hw.turret->Update(
      m_turretPhysics.GetAngularVelocity().value() * kRadPerSecToRpm * constants::kTurretGearing, vbus, dt.value());
  m_turretPhysics.SetInputVoltage(units::volt_t{volts});
  m_turretPhysics.Update(dt);

  // Hard stops
  const double degrees = TurretDegrees();
  if (degrees < constants::kTurretMinDeg || degrees > constants::kTurretMaxDeg) {
    const double clamped = std::clamp(degrees, constants::kTurretMinDeg, constants::kTurretMaxDeg);
    m_turretPhysics.SetState(units::degree_t{clamped}, 0_rad_per_s);
  }

  const double motorRpm =
      m_turretPhysics.GetAngularVelocity().value() * kRadPerSecToRpm * constants::kTurretGearing;
  m_hw.turret->MoveEncoder((TurretDegrees() - degreesBefore) / constants::kTurretDegreesPerMotorRotation, motorRpm);
}

// A ball moves only while EVERY feeder motor (e.g. feeder + kicker) is running.
// They are never loaded enough to matter: each spins at its commanded fraction of free speed.
bool ShooterSim::IsFeeding(double dtSeconds, double vbus) {
  bool allRunning = !m_hw.feeders.empty();
  for (const std::unique_ptr<SimMotor>& feeder : m_hw.feeders) {
    const double freeRpm = feeder->Gearbox().freeSpeed.value() * kRadPerSecToRpm;
    feeder->Update(feeder->AppliedOutput() * freeRpm, vbus, dtSeconds);
    allRunning = allRunning && std::abs(feeder->AppliedOutput()) > constants::kMinFeedOutput;
  }
  return allRunning;
}

void ShooterSim::Launch(const frc::Pose2d& truePose, const frc::ChassisSpeeds& fieldSpeeds) {

  // Turret position on the field, and its velocity (robot velocity + spin x offset)
  const frc::Translation2d offset =
      m_hw.turretOffset.RotateBy(truePose.Rotation());
  const frc::Translation2d turret = truePose.Translation() + offset;
  const double omega = fieldSpeeds.omega.value();
  const double shooterVx = fieldSpeeds.vx.value() - omega * offset.Y().value();
  const double shooterVy = fieldSpeeds.vy.value() + omega * offset.X().value();

  const double fieldYaw = truePose.Rotation().Radians().value() + TurretDegrees() * kDegToRad;
  const double exitSpeed = std::max(0.0, FlywheelRpm()) * constants::kExitSpeedPerRpm;

  FlyingBall flying;
  flying.ball.position = Vec3{turret.X().value(), turret.Y().value(), constants::kLaunchHeightMeters};
  flying.ball.launchPosition = flying.ball.position;
  flying.ball.velocity = LaunchVelocity(exitSpeed, constants::kHoodAngleDeg * kDegToRad, fieldYaw,
                                        shooterVx, shooterVy);
  flying.hub = GoalFor(m_hw.isRed());
  flying.path.push_back(ToPose3d(flying.ball.position));

  ShotRecord record;
  record.launchTimeSeconds = frc::Timer::GetFPGATimestamp().value();
  record.robotPose = truePose;
  record.fieldSpeeds = fieldSpeeds;
  record.flywheelRpm = FlywheelRpm() * constants::kFlywheelGearing;
  record.targetRpm = m_hw.targetRpm ? m_hw.targetRpm() : 0.0;
  record.turretRobotDeg = TurretDegrees();
  record.distanceToHub = turret.Distance(flying.hub.center).value();
  flying.recordIndex = m_records.size();
  m_records.push_back(record);
  m_balls.push_back(std::move(flying));

  // The ball takes energy out of the flywheel
  m_flywheelPhysics.SetVelocity(m_flywheelPhysics.GetAngularVelocity() *
                                (1.0 - frc::SmartDashboard::GetNumber(kLossPerShotKey, constants::kFlywheelSpeedLossPerShot)));
}

void ShooterSim::UpdateBalls(double dtSeconds) {
  const BallPhysics physics = MakeBallPhysics();
  for (auto it = m_balls.begin(); it != m_balls.end();) {
    const std::optional<ShotResult> result = StepBall(it->ball, dtSeconds, it->hub, physics);
    it->path.push_back(ToPose3d(it->ball.position));
    if (result.has_value()) {
      Record(result.value(), it->path);
      m_records[it->recordIndex].finished = true;
      m_records[it->recordIndex].result = result.value();
      if (result->outcome != ShotOutcome::kScored || constants::kReturnScoredBalls) {
        const Landing landing = LandBall(result.value(), it->ball, it->hub, physics);
        m_field.Add(landing.position, landing.velocity);
      }
      it = m_balls.erase(it);
    } else {
      ++it;
    }
  }
}

void ShooterSim::Record(const ShotResult& result, const std::vector<frc::Pose3d>& path) {
  m_stats.finished++;
  m_stats.last = result;
  switch (result.outcome) {
    case ShotOutcome::kScored: m_stats.scored++; break;
    case ShotOutcome::kMissedLong: m_stats.missedLong++; break;
    case ShotOutcome::kMissedShort: m_stats.missedShort++; break;
    case ShotOutcome::kHitHubSide: m_stats.hitHubSide++; break;
    case ShotOutcome::kHitFloor: m_stats.hitFloor++; break;
  }
  m_stats.sumAbsLongShort += std::abs(result.longShortMeters);
  m_stats.sumAbsLeftRight += std::abs(result.leftRightMeters);
  m_lastPathPublisher.Set(path);
}

void ShooterSim::PublishTelemetry() {
  std::vector<frc::Pose3d> balls;
  balls.reserve(m_balls.size());
  for (const FlyingBall& flying : m_balls) {
    balls.push_back(ToPose3d(flying.ball.position));
  }
  m_ballPublisher.Set(balls);
  if (!m_balls.empty()) {
    m_currentPathPublisher.Set(m_balls.back().path);
  }

  const int finished = std::max(m_stats.finished, 1);
  frc::SmartDashboard::PutNumber("Sim/Shots/Fired", m_stats.fired);
  frc::SmartDashboard::PutNumber("Sim/Shots/Scored", m_stats.scored);
  frc::SmartDashboard::PutNumber("Sim/Shots/HitPercent",
                                 m_stats.finished > 0 ? 100.0 * m_stats.scored / m_stats.finished : 0.0);
  frc::SmartDashboard::PutNumber("Sim/Shots/MissedLong", m_stats.missedLong);
  frc::SmartDashboard::PutNumber("Sim/Shots/MissedShort", m_stats.missedShort);
  frc::SmartDashboard::PutNumber("Sim/Shots/HitHubSide", m_stats.hitHubSide);
  frc::SmartDashboard::PutNumber("Sim/Shots/HitFloor", m_stats.hitFloor);
  frc::SmartDashboard::PutString("Sim/Shots/LastResult", std::string{ToString(m_stats.last.outcome)});
  frc::SmartDashboard::PutNumber("Sim/Shots/LastLongShortM", m_stats.last.longShortMeters);
  frc::SmartDashboard::PutNumber("Sim/Shots/LastLeftRightM", m_stats.last.leftRightMeters);
  frc::SmartDashboard::PutNumber("Sim/Shots/LastFlightSec", m_stats.last.flightSeconds);
  frc::SmartDashboard::PutNumber("Sim/Shots/AvgAbsLongShortM", m_stats.sumAbsLongShort / finished);
  frc::SmartDashboard::PutNumber("Sim/Shots/AvgAbsLeftRightM", m_stats.sumAbsLeftRight / finished);
  frc::SmartDashboard::PutNumber("Sim/Shots/BallsInRobot", m_ballsInRobot);
  frc::SmartDashboard::PutNumber("Sim/Flywheel/RPM", FlywheelRpm());
  frc::SmartDashboard::PutNumber("Sim/Turret/RobotRelativeDeg", TurretDegrees());
}

frc::Pose3d ShooterSim::TurretComponent() const {
  return frc::Pose3d{m_hw.turretOffset.X(), m_hw.turretOffset.Y(), 0_m,
                     frc::Rotation3d{0_deg, 0_deg, units::degree_t{TurretDegrees()}}};
}

double ShooterSim::FlywheelRpm() const {
  return m_flywheelPhysics.GetAngularVelocity().value() * kRadPerSecToRpm;
}

double ShooterSim::TurretDegrees() const {
  return units::degree_t{m_turretPhysics.GetAngularPosition()}.value();
}

units::ampere_t ShooterSim::CurrentDraw() const {
  return units::math::abs(m_flywheelPhysics.GetCurrentDraw()) +
         units::math::abs(m_turretPhysics.GetCurrentDraw());
}

}  // namespace robotsim
