#include "sim/DrivetrainSim.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <frc/geometry/Twist2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angular_velocity.h>

#include "sim/SimConstants.h"

namespace robotsim {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kRadPerSecToRpm = 60.0 / kTwoPi;
constexpr double kContactMeters = 1e-4;  // pushed back further than this = touching a wall

// Each module pushes about a quarter of the robot.
units::kilogram_square_meter_t DriveInertia() {
  const double r = constants::kWheelRadiusMeters;
  return units::kilogram_square_meter_t{constants::kRobotMass.value() / 4.0 * r * r};
}

frc::sim::DCMotorSim MakeMotorSim(const frc::DCMotor& gearbox,
                                  units::kilogram_square_meter_t inertia, double gearing) {
  return frc::sim::DCMotorSim{frc::LinearSystemId::DCMotorSystem(gearbox, inertia, gearing), gearbox};
}

}  // namespace

// ---------------- SwerveModuleSim ----------------

SwerveModuleSim::SwerveModuleSim(SwerveModuleHardware hardware)
    : m_hw(std::move(hardware)),
      m_drivePhysics(MakeMotorSim(m_hw.drive->Gearbox(), DriveInertia(), constants::kDriveGearing)),
      m_steerPhysics(MakeMotorSim(m_hw.steer->Gearbox(), constants::kSteerInertia, constants::kSteerGearing)) {}

frc::SwerveModuleState SwerveModuleSim::Update(units::second_t dt, units::volt_t battery) {
  const double vbus = battery.value();
  const double dtSeconds = dt.value();
  const double wheelRadiansBefore = m_drivePhysics.GetAngularPosition().value();
  const double moduleRadiansBefore = m_steerPhysics.GetAngularPosition().value();

  // 1) The motor controllers work out their voltage from what the robot code asked for
  m_drivePhysics.SetInputVoltage(units::volt_t{m_hw.drive->Update(m_driveMotorRpm, vbus, dtSeconds)});
  m_steerPhysics.SetInputVoltage(units::volt_t{m_hw.steer->Update(m_steerMotorRpm, vbus, dtSeconds)});

  // 2) Motor physics
  m_drivePhysics.Update(dt);
  m_steerPhysics.Update(dt);

  // 3) Write the sensors exactly as the robot code reads them:
  //    motor encoders in MOTOR rotations / RPM, steering encoder in module rotations.
  const double wheelRotationsMoved =
      (m_drivePhysics.GetAngularPosition().value() - wheelRadiansBefore) / kTwoPi;
  const double wheelRadPerSec = m_drivePhysics.GetAngularVelocity().value();
  m_driveMotorRpm = wheelRadPerSec * kRadPerSecToRpm * constants::kDriveGearing;
  m_hw.drive->MoveEncoder(wheelRotationsMoved * constants::kDriveGearing, m_driveMotorRpm);

  const units::radian_t moduleAngle = m_steerPhysics.GetAngularPosition();
  const units::radians_per_second_t moduleRate = m_steerPhysics.GetAngularVelocity();
  m_steerMotorRpm = moduleRate.value() * kRadPerSecToRpm * constants::kSteerGearing;
  m_hw.steer->MoveEncoder((moduleAngle.value() - moduleRadiansBefore) / kTwoPi * constants::kSteerGearing,
                          m_steerMotorRpm);
  if (m_hw.steerEncoder) {
    m_hw.steerEncoder->Set(units::turn_t{moduleAngle}.value(), moduleRate.value() / kTwoPi, vbus);
  }

  return frc::SwerveModuleState{
      units::meters_per_second_t{wheelRadPerSec * constants::kWheelRadiusMeters},
      frc::Rotation2d{moduleAngle}};
}

void SwerveModuleSim::Accelerate(double metersPerSecondSquared, units::second_t dt) {
  const double wheelRadPerSec = m_drivePhysics.GetAngularVelocity().value() +
                                metersPerSecondSquared * dt.value() / constants::kWheelRadiusMeters;
  m_drivePhysics.SetState(m_drivePhysics.GetAngularPosition(), units::radians_per_second_t{wheelRadPerSec});
  m_driveMotorRpm = wheelRadPerSec * kRadPerSecToRpm * constants::kDriveGearing;
}

units::ampere_t SwerveModuleSim::CurrentDraw() const {
  return units::math::abs(m_drivePhysics.GetCurrentDraw()) +
         units::math::abs(m_steerPhysics.GetCurrentDraw());
}

// ---------------- DrivetrainSim ----------------

DrivetrainSim::DrivetrainSim(DrivetrainHardware hardware)
    : m_kinematics(hardware.kinematics),
      m_gyro(std::move(hardware.gyro)),
      m_pose(constants::kStartPose),
      m_obstacles(RobotObstacles(FieldOptions{constants::kRobotCanCrossBumps, constants::kRobotFitsUnderTrench})) {
  for (size_t i = 0; i < m_modules.size(); ++i) {
    m_modules[i] = std::make_unique<SwerveModuleSim>(std::move(hardware.modules[i]));
  }
  m_gyro->SetYaw(m_pose.Rotation().Degrees().value());
}

void DrivetrainSim::Update(units::second_t dt, units::volt_t battery) {
  wpi::array<frc::SwerveModuleState, 4> states{wpi::empty_array};
  for (size_t i = 0; i < m_modules.size(); ++i) {
    states[i] = ApplySlope(i, m_modules[i]->Update(dt, battery), dt);
  }
  m_robotSpeeds = m_kinematics.ToChassisSpeeds(states);

  const double seconds = dt.value();
  m_pose = m_pose.Exp(frc::Twist2d{m_robotSpeeds.vx * dt, m_robotSpeeds.vy * dt,
                                   m_robotSpeeds.omega * dt});

  // Walls, hubs, towers (and bumps / trenches if the robot can't use them): the
  // bumpers stop there, but the wheels keep spinning, so odometry drifts, just
  // like pushing against the real thing.
  const frc::Pose2d unblocked = m_pose;
  m_pose = ResolveRobotCollisions(m_pose, constants::kBumperLengthMeters / 2.0,
                                  constants::kBumperWidthMeters / 2.0, m_obstacles);
  // Contact = the bumpers had to be pushed back out of something this loop
  const bool touching = m_pose.Translation().Distance(unblocked.Translation()).value() > kContactMeters;
  if (touching && !m_inContact) {
    m_collisions++;
    if (!m_firstContact) {
      m_firstContact = m_pose;
    }
  }
  if (touching) {
    m_contactSeconds += seconds;
  }
  m_inContact = touching;
  UpdateTilt();

  constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  m_batteryVolts = battery.value();
  m_gyro->AddYaw(m_robotSpeeds.omega.value() * seconds * kRadToDeg, m_robotSpeeds.omega.value() * kRadToDeg,
                 m_batteryVolts);
}

void DrivetrainSim::ResetPose(const frc::Pose2d& pose) {
  m_gyro->AddYaw((pose.Rotation() - m_pose.Rotation()).Degrees().value(), 0.0, m_batteryVolts);
  m_pose = pose;
}

// On a ramp, gravity pulls each wheel back (uphill) or forward (downhill), and a
// wheel that rolls d meters along the ramp only covers d*cos(angle) of ground,
// so odometry (which counts wheel turns) drifts a little, like a real robot.
frc::SwerveModuleState DrivetrainSim::ApplySlope(size_t module, frc::SwerveModuleState state, units::second_t dt) {
  const frc::Translation2d where =
      m_pose.Translation() + m_kinematics.GetModules()[module].RotateBy(m_pose.Rotation());
  const frc::Translation2d slope = FloorSlopeAt(where);
  if (slope.Norm().value() < 1e-6) {
    return state;
  }
  const frc::Rotation2d rolling = m_pose.Rotation() + state.angle;
  const double rise = slope.X().value() * rolling.Cos() + slope.Y().value() * rolling.Sin();  // dz per m rolled
  const double cosAngle = 1.0 / std::sqrt(1.0 + rise * rise);
  const double acceleration = -constants::kGravity * rise * cosAngle;  // g * sin(angle), downhill
  m_modules[module]->Accelerate(acceleration, dt);
  state.speed = units::meters_per_second_t{(state.speed.value() + acceleration * dt.value()) * cosAngle};
  return state;
}

// Height and tilt from where the four wheels sit (a plane fitted through them).
void DrivetrainSim::UpdateTilt() {
  const auto& modules = m_kinematics.GetModules();
  std::array<double, 4> heights{};
  double meanHeight = 0.0;
  for (size_t i = 0; i < modules.size(); ++i) {
    heights[i] = FloorHeightAt(m_pose.Translation() + modules[i].RotateBy(m_pose.Rotation()));
    meanHeight += heights[i] / modules.size();
  }
  double sumXH = 0.0, sumXX = 0.0, sumYH = 0.0, sumYY = 0.0;
  for (size_t i = 0; i < modules.size(); ++i) {
    const double x = modules[i].X().value();
    const double y = modules[i].Y().value();
    sumXH += x * (heights[i] - meanHeight);
    sumXX += x * x;
    sumYH += y * (heights[i] - meanHeight);
    sumYY += y * y;
  }
  m_heightMeters = meanHeight;
  m_pitchRadians = sumXX > 0.0 ? -std::atan(sumXH / sumXX) : 0.0;  // front higher = nose up = - pitch
  m_rollRadians = sumYY > 0.0 ? std::atan(sumYH / sumYY) : 0.0;    // left higher = + roll
  constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  m_gyro->SetTilt(m_pitchRadians * kRadToDeg, m_rollRadians * kRadToDeg);
}

frc::Pose3d DrivetrainSim::GetPose3d() const {
  return frc::Pose3d{m_pose.X(), m_pose.Y(), units::meter_t{m_heightMeters},
                     frc::Rotation3d{units::radian_t{m_rollRadians}, units::radian_t{m_pitchRadians},
                                     m_pose.Rotation().Radians()}};
}

frc::ChassisSpeeds DrivetrainSim::GetFieldRelativeSpeeds() const {
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_robotSpeeds, m_pose.Rotation());
}

units::ampere_t DrivetrainSim::CurrentDraw() const {
  units::ampere_t total{0};
  for (const auto& module : m_modules) {
    total += module->CurrentDraw();
  }
  return total;
}

}  // namespace robotsim
