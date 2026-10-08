// ================================================================
// Starts the simulator without changing the robot code's loop.
//
// The robot project sets g_simRobotContainer in Robot::SimulationInit().
// WPILib calls this hook after every SimulationPeriodic() (every 20 ms);
// the first call builds the simulated robot, later calls step it.
// ================================================================

#include <hal/simulation/MockHooks.h>
#include <units/time.h>

#include "RobotContainer.h"
#include "sim/RobotSim.h"
#include "sim/robot/RobotBinding.h"

// Defined in the robot project's Robot.cpp
extern RobotContainer* g_simRobotContainer;

namespace {

constexpr units::second_t kLoopPeriod = 20_ms;  // TimedRobot default period

void OnSimPeriodic(void*) {
  if (g_simRobotContainer == nullptr) {
    return;  // robot code not running (e.g. unit tests)
  }
  // Lives until the program exits and is never destroyed on purpose:
  // at exit the subsystems it points to are already gone.
  static robotsim::RobotSim* robotSim = robotsim::binding::MakeRobotSim(*g_simRobotContainer).release();
  robotSim->Update(kLoopPeriod);
}

const int32_t kSimHookId = HALSIM_RegisterSimPeriodicAfterCallback(&OnSimPeriodic, nullptr);

}  // namespace
