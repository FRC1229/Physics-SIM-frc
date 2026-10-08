#include "sim/IntakeSim.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <frc/system/plant/LinearSystemId.h>

#include "sim/SimConstants.h"

namespace robotsim {

namespace {

constexpr double kRadPerSecToRpm = 60.0 / (2.0 * std::numbers::pi);
constexpr double kMaxPickupCredit = 2.0;  // the roller can hold at most this many balls "in the grab"

IntakeZone MakeIntakeZone() {
  const double bumperEdge = constants::kBumperLengthMeters / 2.0;
  const double nearX = bumperEdge - constants::kBallDiameterMeters / 2.0;
  const double farX = bumperEdge + constants::kIntakeReachMeters;
  // Robot frame: +x = front. An intake on the back grabs behind the robot.
  return constants::kIntakeAtBack ? IntakeZone{-farX, -nearX, constants::kIntakeHalfWidthMeters}
                                  : IntakeZone{nearX, farX, constants::kIntakeHalfWidthMeters};
}

}  // namespace

IntakeSim::IntakeSim(std::unique_ptr<SimMotor> armMotor, std::unique_ptr<SimMotor> rollerMotor)
    : m_armMotor(std::move(armMotor)),
      m_rollerMotor(std::move(rollerMotor)),
      m_armPhysics(frc::LinearSystemId::DCMotorSystem(m_armMotor->Gearbox(), constants::kIntakeArmInertia,
                                                      constants::kIntakeGearing),
                   m_armMotor->Gearbox()) {}

void IntakeSim::Update(units::second_t dt, units::volt_t battery) {
  const double vbus = battery.value();
  const double dtSeconds = dt.value();

  const double degreesBefore = ArmDegrees();
  const double armVolts = m_armMotor->Update(
      m_armPhysics.GetAngularVelocity().value() * kRadPerSecToRpm * constants::kIntakeGearing, vbus, dtSeconds);
  m_armPhysics.SetInputVoltage(units::volt_t{armVolts});
  m_armPhysics.Update(dt);

  // Hard stops: stowed against the frame, or resting on the carpet
  const double degrees = ArmDegrees();
  if (degrees < constants::kIntakeMinDeg || degrees > constants::kIntakeMaxDeg) {
    const double clamped = std::clamp(degrees, constants::kIntakeMinDeg, constants::kIntakeMaxDeg);
    m_armPhysics.SetState(units::degree_t{clamped}, 0_rad_per_s);
  }

  // The arm motor's encoder (motor rotations)
  m_armMotor->MoveEncoder((ArmDegrees() - degreesBefore) / 360.0 * constants::kIntakeGearing,
                          m_armPhysics.GetAngularVelocity().value() * kRadPerSecToRpm * constants::kIntakeGearing);

  // The roller is never loaded enough to matter: it spins at its commanded fraction of free speed
  const double rollerFreeRpm = m_rollerMotor->Gearbox().freeSpeed.value() * kRadPerSecToRpm;
  m_rollerMotor->Update(m_rollerMotor->AppliedOutput() * rollerFreeRpm, vbus, dtSeconds);
}

int IntakeSim::Collect(FieldBalls& field, const frc::Pose2d& truePose, int room, units::second_t dt) {
  if (!IsIntaking() || room <= 0) {
    m_pickupCredit = 0.0;
    return 0;
  }
  m_pickupCredit = std::min(m_pickupCredit + constants::kIntakeBallsPerSecond * dt.value(), kMaxPickupCredit);
  const int canTake = std::min(static_cast<int>(m_pickupCredit), room);
  const int taken = field.Collect(truePose, MakeIntakeZone(), canTake);
  m_pickupCredit -= taken;
  return taken;
}

void IntakeSim::ResetArm() {
  const double degrees = ArmDegrees();
  m_armPhysics.SetState(0_rad, 0_rad_per_s);
  m_armMotor->MoveEncoder(-degrees / 360.0 * constants::kIntakeGearing, 0.0);
  m_pickupCredit = 0.0;
}

bool IntakeSim::IsIntaking() const {
  return ArmDegrees() >= constants::kIntakeDeployedDeg &&
         std::abs(m_rollerMotor->AppliedOutput()) > constants::kMinRollerOutput;
}

double IntakeSim::ArmDegrees() const {
  return units::degree_t{m_armPhysics.GetAngularPosition()}.value();
}

frc::Pose3d IntakeSim::ArmComponent() const {
  // + pitch tips the arm's far end forward and down
  // Arm on the back: pivot behind center, and it folds out the other way
  const double side = constants::kIntakeAtBack ? -1.0 : 1.0;
  return frc::Pose3d{units::meter_t{side * constants::kIntakePivotForwardMeters}, 0_m,
                     units::meter_t{constants::kIntakePivotUpMeters},
                     frc::Rotation3d{0_deg, units::degree_t{side * ArmDegrees()}, 0_deg}};
}

units::ampere_t IntakeSim::CurrentDraw() const {
  return units::math::abs(m_armPhysics.GetCurrentDraw());
}

}  // namespace robotsim
