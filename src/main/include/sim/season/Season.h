#pragma once

// ================================================================
// What every season provides (implemented in src/main/cpp/sim/season/SeasonYYYY.cpp):
//
//   RobotObstacles(options)  - solid shapes the robot bumps into     (sim/FieldGeometry.h)
//   BallObstacles()          - solid shapes rolling pieces bounce off (sim/FieldGeometry.h)
//   StartingFieldBalls()     - where game pieces start               (sim/FieldBalls.h)
//   GoalFor(isRed)           - the goal shots are aimed at           (below)
//
// plus the numbers in sim/season/SeasonYYYY.h.
// ================================================================

#include "sim/Ballistics.h"

namespace robotsim {

// The goal (hub) a robot on this alliance shoots into.
Hub GoalFor(bool isRed);

}  // namespace robotsim
