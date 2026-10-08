#include "sim/Ballistics.h"

#include <algorithm>
#include <cmath>

namespace robotsim {

namespace {

constexpr double kSubStepSeconds = 0.002;
constexpr double kMaxFlightSeconds = 6.0;

double HorizontalDistance(const Vec3& point, const frc::Translation2d& center) {
  return std::hypot(point.x - center.X().value(), point.y - center.Y().value());
}

// Splits the hub->point offset into along-the-shot and sideways parts.
ShotResult MakeResult(ShotOutcome outcome, const Vec3& point, const Ball& ball, const Hub& hub) {
  const double hubX = hub.center.X().value();
  const double hubY = hub.center.Y().value();
  double dirX = hubX - ball.launchPosition.x;
  double dirY = hubY - ball.launchPosition.y;
  const double length = std::hypot(dirX, dirY);
  if (length > 1e-9) {
    dirX /= length;
    dirY /= length;
  } else {
    dirX = 1.0;
    dirY = 0.0;
  }
  const double offX = point.x - hubX;
  const double offY = point.y - hubY;

  ShotResult result;
  result.outcome = outcome;
  result.flightSeconds = ball.flightSeconds;
  result.longShortMeters = offX * dirX + offY * dirY;
  result.leftRightMeters = dirX * offY - dirY * offX;
  result.endPosition = frc::Translation3d{units::meter_t{point.x}, units::meter_t{point.y},
                                         units::meter_t{point.z}};
  return result;
}

ShotOutcome MissDirection(const ShotResult& result) {
  return result.longShortMeters >= 0.0 ? ShotOutcome::kMissedLong : ShotOutcome::kMissedShort;
}

void Integrate(Ball& ball, double dt, const BallPhysics& physics) {
  Vec3& v = ball.velocity;
  const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  const double drag = physics.dragFactor * speed;
  v.x += -drag * v.x * dt;
  v.y += -drag * v.y * dt;
  v.z += (-physics.gravity - drag * v.z) * dt;
  ball.position.x += v.x * dt;
  ball.position.y += v.y * dt;
  ball.position.z += v.z * dt;
  ball.flightSeconds += dt;
}

// Moves a point that is inside (or touching) the hub body out to its edge.
frc::Translation2d OutsideHub(double x, double y, const Hub& hub, double ballRadius) {
  const double minDistance = hub.outerRadius + ballRadius;
  double offX = x - hub.center.X().value();
  double offY = y - hub.center.Y().value();
  const double distance = std::hypot(offX, offY);
  if (distance >= minDistance) {
    return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
  }
  if (distance < 1e-9) {
    offX = -1.0;  // dead center: pick a side
    offY = 0.0;
  } else {
    offX /= distance;
    offY /= distance;
  }
  return frc::Translation2d{units::meter_t{hub.center.X().value() + offX * minDistance},
                            units::meter_t{hub.center.Y().value() + offY * minDistance}};
}

// Keeps flying a copy of the ball (ignoring the hub) until it reaches the carpet.
Ball FlyToCarpet(Ball ball, const BallPhysics& physics) {
  while (ball.position.z > physics.ballRadius && ball.flightSeconds < kMaxFlightSeconds) {
    Integrate(ball, kSubStepSeconds, physics);
  }
  return ball;
}

frc::Translation2d Horizontal(const Vec3& v, double scale) {
  return frc::Translation2d{units::meter_t{v.x * scale}, units::meter_t{v.y * scale}};
}

// Bounces a carpet velocity off the hub body if it is heading into the hub.
frc::Translation2d BounceOffHub(const frc::Translation2d& position, const frc::Translation2d& velocity,
                                const Hub& hub) {
  const frc::Translation2d out = position - hub.center;
  const double length = out.Norm().value();
  if (length < 1e-9) {
    return velocity;
  }
  const double nx = out.X().value() / length;
  const double ny = out.Y().value() / length;
  const double into = velocity.X().value() * nx + velocity.Y().value() * ny;
  if (into >= 0.0) {
    return velocity;
  }
  return frc::Translation2d{velocity.X() - units::meter_t{2.0 * into * nx},
                            velocity.Y() - units::meter_t{2.0 * into * ny}};
}

}  // namespace

std::string_view ToString(ShotOutcome outcome) {
  switch (outcome) {
    case ShotOutcome::kScored: return "Scored";
    case ShotOutcome::kMissedLong: return "Missed long";
    case ShotOutcome::kMissedShort: return "Missed short";
    case ShotOutcome::kHitHubSide: return "Hit hub side";
    case ShotOutcome::kHitFloor: return "Hit floor";
  }
  return "Unknown";
}

Vec3 LaunchVelocity(double exitSpeed, double hoodAngleRad, double fieldYawRad,
                    double shooterVx, double shooterVy) {
  const double horizontal = exitSpeed * std::cos(hoodAngleRad);
  return Vec3{horizontal * std::cos(fieldYawRad) + shooterVx,
              horizontal * std::sin(fieldYawRad) + shooterVy,
              exitSpeed * std::sin(hoodAngleRad)};
}

std::optional<ShotResult> StepBall(Ball& ball, double dt, const Hub& hub,
                                   const BallPhysics& physics) {
  double remaining = dt;
  while (remaining > 1e-12) {
    const double step = std::min(kSubStepSeconds, remaining);
    remaining -= step;

    const Vec3 previous = ball.position;
    Integrate(ball, step, physics);
    const Vec3& now = ball.position;

    // Coming down through the opening height: in the hub, or a miss.
    if (previous.z >= hub.openingHeight && now.z < hub.openingHeight) {
      const double fraction = (previous.z - hub.openingHeight) / (previous.z - now.z);
      const Vec3 crossing{previous.x + (now.x - previous.x) * fraction,
                          previous.y + (now.y - previous.y) * fraction, hub.openingHeight};
      const double distance = HorizontalDistance(crossing, hub.center);
      if (distance <= hub.openingRadius - physics.ballRadius) {
        return MakeResult(ShotOutcome::kScored, crossing, ball, hub);
      }
      ShotResult miss = MakeResult(ShotOutcome::kMissedLong, crossing, ball, hub);
      miss.outcome = MissDirection(miss);
      return miss;
    }

    // Below the opening and touching the hub body: bounced off the side.
    if (now.z < hub.openingHeight &&
        HorizontalDistance(now, hub.center) <= hub.outerRadius + physics.ballRadius) {
      return MakeResult(ShotOutcome::kHitHubSide, now, ball, hub);
    }

    // Never got up to the opening height.
    if (now.z <= physics.ballRadius || ball.flightSeconds > kMaxFlightSeconds) {
      return MakeResult(ShotOutcome::kHitFloor, now, ball, hub);
    }
  }
  return std::nullopt;
}

Landing LandBall(const ShotResult& result, const Ball& ball, const Hub& hub, const BallPhysics& physics) {
  const double kept = physics.landingSpeedKept;
  switch (result.outcome) {
    case ShotOutcome::kScored: {
      const frc::Translation2d away = hub.exitPoint - hub.center;
      const double length = away.Norm().value();
      const frc::Translation2d velocity =
          length > 1e-9 ? away * (hub.exitSpeed / length) : frc::Translation2d{};
      return Landing{hub.exitPoint, velocity};
    }
    case ShotOutcome::kMissedLong:
    case ShotOutcome::kMissedShort: {
      const Ball landed = FlyToCarpet(ball, physics);
      const frc::Translation2d position = OutsideHub(landed.position.x, landed.position.y, hub, physics.ballRadius);
      return Landing{position, BounceOffHub(position, Horizontal(landed.velocity, kept), hub)};
    }
    case ShotOutcome::kHitHubSide:
    case ShotOutcome::kHitFloor:
      break;
  }
  const frc::Translation2d position = OutsideHub(result.endPosition.X().value(), result.endPosition.Y().value(),
                                                 hub, physics.ballRadius);
  return Landing{position, BounceOffHub(position, Horizontal(ball.velocity, kept), hub)};
}

}  // namespace robotsim
