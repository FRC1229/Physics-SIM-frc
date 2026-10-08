#pragma once

// ================================================================
// Solid parts of the 2026 field (pure math, no hardware, no NetworkTables).
// Pushes the robot's bumpers and rolling balls back out of the walls,
// hubs, towers (and bumps / trenches when the robot can't use them).
// ================================================================

#include <optional>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

namespace robotsim {

// Axis-aligned rectangle on the field (meters, blue-origin field frame).
struct Box {
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
};

struct FieldOptions {
  bool robotCanCrossBumps = true;    // false: the bumps are walls for the robot
  bool robotFitsUnderTrench = true;  // false: the trench arms are walls for the robot
};

// What the robot's bumpers bump into: hubs, towers (+ bumps, trench arms if the robot can't use them).
std::vector<Box> RobotObstacles(const FieldOptions& options);

// What rolling balls bounce off: hubs, towers, trench supports (balls roll over bumps and under trenches).
std::vector<Box> BallObstacles();

// How high the floor is here (0 on the carpet, more on a bump or ramp). Per season.
double FloorHeightAt(const frc::Translation2d& point);

// Which way and how steeply the floor rises here: (dz/dx, dz/dy). Zero on flat carpet.
frc::Translation2d FloorSlopeAt(const frc::Translation2d& point);

// The robot (a rectangle: bumper half-length forward, half-width sideways)
// moved the shortest way out of every obstacle and back inside the field walls.
frc::Pose2d ResolveRobotCollisions(const frc::Pose2d& pose, double halfLength, double halfWidth,
                                   const std::vector<Box>& obstacles);

// How a circle overlaps something: push it `depth` meters along (normalX, normalY).
struct Contact {
  double normalX = 0.0;
  double normalY = 0.0;
  double depth = 0.0;
};

// Circle against a box (normal points out of the box).
std::optional<Contact> CircleVsBox(const frc::Translation2d& center, double radius, const Box& box);

// Circle against the robot's bumper rectangle (normal points away from the robot).
std::optional<Contact> CircleVsRobot(const frc::Translation2d& center, double radius,
                                     const frc::Pose2d& robotPose, double halfLength, double halfWidth);

}  // namespace robotsim
