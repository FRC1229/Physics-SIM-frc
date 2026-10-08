#pragma once

// ================================================================
// Intake arm + roller physics for the simulator (any robot).
// The arm runs real motor physics from the voltage the robot code sends
// to its motor, and writes back that motor's encoder. When the arm is down
// and the roller spins, balls in front of the bumper are picked up off the field.
//
// Results on the dashboard under Sim/Intake/...
// ================================================================

#include <memory>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "sim/FieldBalls.h"
#include "sim/hardware/SimHardware.h"

namespace robotsim {

class IntakeSim {
 public:
  IntakeSim(std::unique_ptr<SimMotor> armMotor, std::unique_ptr<SimMotor> rollerMotor);

  void Update(units::second_t dt, units::volt_t battery);

  // Picks up balls within reach, if the arm is down and the roller is spinning.
  // `room` = how many more balls the robot can hold. Returns how many it picked up.
  int Collect(FieldBalls& field, const frc::Pose2d& truePose, int room, units::second_t dt);

  // Back to stowed (like a fresh power-on): arm at 0, its encoder moved back with it
  void ResetArm();

  bool IsIntaking() const;
  double ArmDegrees() const;  // 0 = stowed, increasing as the arm lowers
  frc::Pose3d ArmComponent() const;  // robot-relative pose for AdvantageScope
  units::ampere_t CurrentDraw() const;

 private:
  std::unique_ptr<SimMotor> m_armMotor;
  std::unique_ptr<SimMotor> m_rollerMotor;

  frc::sim::DCMotorSim m_armPhysics;  // output = arm angle

  double m_pickupCredit = 0.0;  // balls the roller can grab right now (limits the intake rate)
};

}  // namespace robotsim
