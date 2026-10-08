#pragma once

// ================================================================
// Balls (FUEL) on the carpet (pure logic, no hardware, no NetworkTables).
// They roll and slow down, bounce off the walls, hubs, towers and each
// other, and get shoved by the robot's bumpers. The intake picks them up;
// missed and scored shots put them back down (still rolling).
// ================================================================

#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "sim/FieldGeometry.h"

namespace robotsim {

// Where the intake grabs balls, in the ROBOT frame (x forward, y left).
// A ball is collected when its center is inside this rectangle.
struct IntakeZone {
  double nearX = 0.0;   // closest edge, meters forward of robot center (the bumper)
  double farX = 0.0;    // farthest reach of the intake
  double halfWidth = 0.0;
};

// The robot as the balls see it: its bumper rectangle and how it is moving.
struct RobotBody {
  frc::Pose2d pose;
  frc::ChassisSpeeds fieldSpeeds;  // field-relative
  double halfLength = 0.0;
  double halfWidth = 0.0;
};

class FieldBalls {
 public:
  FieldBalls();

  void Reset(std::vector<frc::Translation2d> balls);
  void Clear();

  // Drops a ball here (kept inside the field walls), optionally already rolling.
  void Add(const frc::Translation2d& position, const frc::Translation2d& velocity = {});

  // Removes up to maxCount balls inside the intake zone, closest to the robot
  // first. Returns how many were picked up.
  int Collect(const frc::Pose2d& robotPose, const IntakeZone& zone, int maxCount);

  // Moves every ball forward by dt: rolling, friction and all collisions.
  void Step(double dtSeconds, const RobotBody& robot);

  const std::vector<frc::Translation2d>& Positions() const { return m_positions; }
  const std::vector<frc::Translation2d>& Velocities() const { return m_velocities; }
  int Count() const { return static_cast<int>(m_positions.size()); }
  // Goes up whenever a ball is added, removed or moves (telemetry only resends on a change)
  unsigned Version() const { return m_version; }

 private:
  void SubStep(double dt, const RobotBody& robot);
  bool Integrate(double dt);
  bool PushedByRobot(const RobotBody& robot);
  void BounceOffStructures();
  void BounceOffEachOther();

  std::vector<frc::Translation2d> m_positions;
  std::vector<frc::Translation2d> m_velocities;  // m/s, same order as m_positions
  std::vector<Box> m_obstacles;
  unsigned m_version = 0;
  bool m_unsettled = false;  // balls were added/reset and may overlap something
};

// Balls on the field at the start of a match (neutral zone pile + both depots).
std::vector<frc::Translation2d> StartingFieldBalls();

}  // namespace robotsim
