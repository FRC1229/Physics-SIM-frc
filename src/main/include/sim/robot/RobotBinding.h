#pragma once

// ================================================================
// ROBOT BINDING: the only place that knows our robot code's classes
// (RobotContainer, its subsystems and their motors). It hands each generic
// mechanism sim the robot's real hardware objects.
//
// New robot next season? Rewrite this file (and RobotConfig.h); the
// mechanism sims, field, vision and tests stay the same.
// ================================================================

#include <memory>

#include "RobotContainer.h"
#include "sim/DrivetrainSim.h"
#include "sim/RobotSim.h"
#include "sim/ShooterSim.h"
#include "sim/hardware/SimHardware.h"

namespace robotsim::binding {

std::unique_ptr<SimMotor> MakeIntakeArmMotor(RobotContainer& robot);
std::unique_ptr<SimMotor> MakeIntakeRollerMotor(RobotContainer& robot);
DrivetrainHardware MakeDrivetrainHardware(RobotContainer& robot);
ShooterHardware MakeShooterHardware(RobotContainer& robot);
RobotCodeHooks MakeRobotCodeHooks(RobotContainer& robot);

// The whole simulated robot for this robot code.
std::unique_ptr<RobotSim> MakeRobotSim(RobotContainer& robot);

}  // namespace robotsim::binding
