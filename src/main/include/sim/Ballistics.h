#pragma once

// ================================================================
// Ball flight physics (pure math, no hardware, no NetworkTables).
// Gravity + air drag, and checks whether a ball drops into the hub.
// ================================================================

#include <optional>
#include <string_view>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

namespace robotsim {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Ball {
  Vec3 position;          // field frame, meters (z = height above carpet)
  Vec3 velocity;          // field frame, m/s
  Vec3 launchPosition;    // where it left the shooter
  double flightSeconds = 0.0;
};

struct Hub {
  frc::Translation2d center;
  double openingHeight = 0.0;  // height the ball must drop through
  double openingRadius = 0.0;  // ball center must be inside (radius - ball radius)
  double outerRadius = 0.0;    // hub body: below the opening, balls bounce off
  frc::Translation2d exitPoint;  // where scored balls come back out onto the carpet
  double exitSpeed = 0.0;         // how fast they roll out (away from the hub center), m/s
};

enum class ShotOutcome { kScored, kMissedLong, kMissedShort, kHitHubSide, kHitFloor };

std::string_view ToString(ShotOutcome outcome);

struct ShotResult {
  ShotOutcome outcome = ShotOutcome::kHitFloor;
  double flightSeconds = 0.0;
  // Where the ball crossed the opening height (or landed), relative to the hub
  // center, split along the shot direction:
  double longShortMeters = 0.0;  // + = past the hub center, - = short
  double leftRightMeters = 0.0;  // + = left of the hub (as seen from the shooter)
  frc::Translation3d endPosition;
};

struct BallPhysics {
  double gravity = 9.81;
  double dragFactor = 0.0;    // drag acceleration = dragFactor * speed^2
  double ballRadius = 0.075;
  double landingSpeedKept = 1.0;  // fraction of horizontal speed kept when the ball lands / bounces
};

// Launch velocity from flywheel exit speed, hood angle, turret direction
// (field angle, radians), plus the velocity of the shooter itself.
Vec3 LaunchVelocity(double exitSpeed, double hoodAngleRad, double fieldYawRad,
                    double shooterVx, double shooterVy);

// Advances one ball by dt (internally sub-stepped). Returns a result once the
// ball has finished (scored or missed); nullopt while it is still flying.
std::optional<ShotResult> StepBall(Ball& ball, double dt, const Hub& hub,
                                   const BallPhysics& physics);

// Where a finished ball comes down on the carpet, and how it is still rolling:
//   scored        - out of the hub's exit point, rolling away from the hub
//   missed        - keeps flying (past the opening) until it lands, then rolls on
//   hit hub side  - drops at the foot of the hub, bounced back off it
//   hit floor     - where it hit, rolling on
// Never inside the hub body.
struct Landing {
  frc::Translation2d position;
  frc::Translation2d velocity;  // m/s along the carpet
};
Landing LandBall(const ShotResult& result, const Ball& ball, const Hub& hub, const BallPhysics& physics);

}  // namespace robotsim
