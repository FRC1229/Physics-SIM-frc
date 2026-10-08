#include "sim/FieldGeometry.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "sim/SimConstants.h"

namespace robotsim {

namespace {

constexpr int kCollisionPasses = 3;  // pushing out of one box can push into another

// Separating-axis test between the robot rectangle and a box. Returns how far
// (and which way) to move the robot to just touch it, or nullopt if apart.
std::optional<frc::Translation2d> RobotPushOut(double centerX, double centerY, double cosA, double sinA,
                                               double halfLength, double halfWidth, const Box& box) {
  const double boxCenterX = (box.minX + box.maxX) / 2.0;
  const double boxCenterY = (box.minY + box.maxY) / 2.0;
  const double boxHalfX = (box.maxX - box.minX) / 2.0;
  const double boxHalfY = (box.maxY - box.minY) / 2.0;
  const double dx = centerX - boxCenterX;
  const double dy = centerY - boxCenterY;

  const std::array<std::array<double, 2>, 4> axes{{{1.0, 0.0}, {0.0, 1.0}, {cosA, sinA}, {-sinA, cosA}}};
  double bestOverlap = 1e9;
  double pushX = 0.0;
  double pushY = 0.0;
  for (const auto& axis : axes) {
    const double nx = axis[0];
    const double ny = axis[1];
    const double robotReach = halfLength * std::abs(cosA * nx + sinA * ny) +
                              halfWidth * std::abs(-sinA * nx + cosA * ny);
    const double boxReach = boxHalfX * std::abs(nx) + boxHalfY * std::abs(ny);
    const double distance = dx * nx + dy * ny;
    const double overlap = robotReach + boxReach - std::abs(distance);
    if (overlap <= 0.0) {
      return std::nullopt;  // found a gap: not touching
    }
    if (overlap < bestOverlap) {
      bestOverlap = overlap;
      const double sign = distance >= 0.0 ? 1.0 : -1.0;
      pushX = sign * nx * overlap;
      pushY = sign * ny * overlap;
    }
  }
  return frc::Translation2d{units::meter_t{pushX}, units::meter_t{pushY}};
}

// Keeps all four bumper corners inside the field walls.
void KeepInsideWalls(double& centerX, double& centerY, double cosA, double sinA, double halfLength,
                     double halfWidth) {
  const double reachX = halfLength * std::abs(cosA) + halfWidth * std::abs(sinA);
  const double reachY = halfLength * std::abs(sinA) + halfWidth * std::abs(cosA);
  centerX = std::clamp(centerX, reachX, constants::kFieldLengthMeters - reachX);
  centerY = std::clamp(centerY, reachY, constants::kFieldWidthMeters - reachY);
}

// Circle against a rectangle centered at the origin: closest point, or the
// shortest way out if the circle's center is already inside.
std::optional<Contact> CircleVsCenteredRect(double x, double y, double radius, double halfX, double halfY) {
  const double closestX = std::clamp(x, -halfX, halfX);
  const double closestY = std::clamp(y, -halfY, halfY);
  const double offX = x - closestX;
  const double offY = y - closestY;
  const double distanceSquared = offX * offX + offY * offY;
  if (distanceSquared > 1e-12) {
    if (distanceSquared >= radius * radius) {
      return std::nullopt;
    }
    const double distance = std::sqrt(distanceSquared);
    return Contact{offX / distance, offY / distance, radius - distance};
  }
  const double toRight = halfX - x;
  const double toLeft = x + halfX;
  const double toTop = halfY - y;
  const double toBottom = y + halfY;
  const double nearest = std::min({toRight, toLeft, toTop, toBottom});
  if (nearest == toRight) return Contact{1.0, 0.0, toRight + radius};
  if (nearest == toLeft) return Contact{-1.0, 0.0, toLeft + radius};
  if (nearest == toTop) return Contact{0.0, 1.0, toTop + radius};
  return Contact{0.0, -1.0, toBottom + radius};
}

}  // namespace

frc::Pose2d ResolveRobotCollisions(const frc::Pose2d& pose, double halfLength, double halfWidth,
                                   const std::vector<Box>& obstacles) {
  double x = pose.X().value();
  double y = pose.Y().value();
  const double cosA = pose.Rotation().Cos();
  const double sinA = pose.Rotation().Sin();
  for (int pass = 0; pass < kCollisionPasses; ++pass) {
    for (const Box& box : obstacles) {
      if (const auto push = RobotPushOut(x, y, cosA, sinA, halfLength, halfWidth, box)) {
        x += push->X().value();
        y += push->Y().value();
      }
    }
    KeepInsideWalls(x, y, cosA, sinA, halfLength, halfWidth);
  }
  return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, pose.Rotation()};
}

// From the season's height map (central differences, so any floor shape works).
frc::Translation2d FloorSlopeAt(const frc::Translation2d& point) {
  constexpr double kStep = 0.01;
  const frc::Translation2d dx{units::meter_t{kStep}, 0_m};
  const frc::Translation2d dy{0_m, units::meter_t{kStep}};
  const double slopeX = (FloorHeightAt(point + dx) - FloorHeightAt(point - dx)) / (2.0 * kStep);
  const double slopeY = (FloorHeightAt(point + dy) - FloorHeightAt(point - dy)) / (2.0 * kStep);
  return frc::Translation2d{units::meter_t{slopeX}, units::meter_t{slopeY}};
}

std::optional<Contact> CircleVsBox(const frc::Translation2d& center, double radius, const Box& box) {
  const double centerX = (box.minX + box.maxX) / 2.0;
  const double centerY = (box.minY + box.maxY) / 2.0;
  return CircleVsCenteredRect(center.X().value() - centerX, center.Y().value() - centerY, radius,
                              (box.maxX - box.minX) / 2.0, (box.maxY - box.minY) / 2.0);
}

std::optional<Contact> CircleVsRobot(const frc::Translation2d& center, double radius,
                                     const frc::Pose2d& robotPose, double halfLength, double halfWidth) {
  const frc::Translation2d local = (center - robotPose.Translation()).RotateBy(-robotPose.Rotation());
  std::optional<Contact> contact =
      CircleVsCenteredRect(local.X().value(), local.Y().value(), radius, halfLength, halfWidth);
  if (!contact.has_value()) {
    return std::nullopt;
  }
  // Normal back into the field frame
  const frc::Translation2d normal =
      frc::Translation2d{units::meter_t{contact->normalX}, units::meter_t{contact->normalY}}.RotateBy(
          robotPose.Rotation());
  contact->normalX = normal.X().value();
  contact->normalY = normal.Y().value();
  return contact;
}

}  // namespace robotsim
