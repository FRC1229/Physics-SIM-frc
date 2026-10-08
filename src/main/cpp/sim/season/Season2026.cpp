// ================================================================
// SEASON 2026 REBUILT: the field's solid shapes, where the FUEL starts, and the
// hubs (goals). Implements sim/season/Season.h. Only compiled when
// sim/SimConstants.h selects this season (SIM_SEASON_YEAR).
// ================================================================

#include "sim/SimConstants.h"

#if SIM_SEASON_YEAR == 2026

#include <cmath>
#include <vector>

#include "sim/FieldBalls.h"
#include "sim/FieldGeometry.h"
#include "sim/season/Season.h"

namespace robotsim {

namespace {

Box CenteredBox(double centerX, double centerY, double halfX, double halfY) {
  return Box{centerX - halfX, centerY - halfY, centerX + halfX, centerY + halfY};
}

void AddHubs(std::vector<Box>& boxes) {
  for (const frc::Translation2d& hub : {constants::kBlueHubCenter, constants::kRedHubCenter}) {
    boxes.push_back(CenteredBox(hub.X().value(), hub.Y().value(), constants::kHubHalfSizeMeters,
                                constants::kHubHalfSizeMeters));
  }
}

void AddTowers(std::vector<Box>& boxes) {
  const double length = constants::kFieldLengthMeters;
  const double width = constants::kFieldWidthMeters;
  const double y = constants::kTowerCenterYMeters;
  const double halfWidth = constants::kTowerHalfWidthMeters;
  boxes.push_back(Box{0.0, y - halfWidth, constants::kTowerDepthMeters, y + halfWidth});
  // Red is the blue field rotated 180 degrees
  boxes.push_back(Box{length - constants::kTowerDepthMeters, width - y - halfWidth, length, width - y + halfWidth});
}

// Bumps sit between each hub and the trenches, one on each side of the hub.
void AddBumps(std::vector<Box>& boxes) {
  const double half = constants::kHubHalfSizeMeters;
  for (const frc::Translation2d& hub : {constants::kBlueHubCenter, constants::kRedHubCenter}) {
    const double x = hub.X().value();
    const double hubMinY = hub.Y().value() - half;
    const double hubMaxY = hub.Y().value() + half;
    const double depth = constants::kBumpHalfDepthMeters;
    boxes.push_back(Box{x - depth, hubMinY - constants::kBumpLengthMeters, x + depth, hubMinY});
    boxes.push_back(Box{x - depth, hubMaxY, x + depth, hubMaxY + constants::kBumpLengthMeters});
  }
}

// Trench arms run from each side wall in line with the hubs.
void AddTrenches(std::vector<Box>& boxes) {
  const double width = constants::kFieldWidthMeters;
  const double depth = constants::kTrenchHalfDepthMeters;
  for (const frc::Translation2d& hub : {constants::kBlueHubCenter, constants::kRedHubCenter}) {
    const double x = hub.X().value();
    boxes.push_back(Box{x - depth, 0.0, x + depth, constants::kTrenchWidthMeters});
    boxes.push_back(Box{x - depth, width - constants::kTrenchWidthMeters, x + depth, width});
  }
}

// The solid trench supports: between the far end of each bump and the trench lane.
void AddTrenchSupports(std::vector<Box>& boxes) {
  const double width = constants::kFieldWidthMeters;
  const double depth = constants::kBumpHalfDepthMeters;
  for (const frc::Translation2d& hub : {constants::kBlueHubCenter, constants::kRedHubCenter}) {
    const double x = hub.X().value();
    const double lowBumpEnd = hub.Y().value() - constants::kHubHalfSizeMeters - constants::kBumpLengthMeters;
    const double highBumpEnd = hub.Y().value() + constants::kHubHalfSizeMeters + constants::kBumpLengthMeters;
    boxes.push_back(Box{x - depth, constants::kTrenchWidthMeters, x + depth, lowBumpEnd});
    boxes.push_back(Box{x - depth, highBumpEnd, x + depth, width - constants::kTrenchWidthMeters});
  }
}

// Height of a bump at this distance from its center line (along the field length).
double BumpProfile(double fromCenter) {
  const double half = constants::kBumpHalfDepthMeters;
  const double ramp = constants::kBumpRampRunMeters;
  if (fromCenter >= half) {
    return 0.0;
  }
  if (fromCenter <= half - ramp) {
    return constants::kBumpHeightMeters;
  }
  return constants::kBumpHeightMeters * (half - fromCenter) / ramp;
}

// Lays out a columns x rows grid of balls centered on `center`.
void AddGrid(std::vector<frc::Translation2d>& balls, const frc::Translation2d& center, int columns,
             int rows, double spacing) {
  const double startX = center.X().value() - (columns - 1) * spacing / 2.0;
  const double startY = center.Y().value() - (rows - 1) * spacing / 2.0;
  for (int column = 0; column < columns; ++column) {
    for (int row = 0; row < rows; ++row) {
      balls.emplace_back(units::meter_t{startX + column * spacing}, units::meter_t{startY + row * spacing});
    }
  }
}

}  // namespace

std::vector<Box> RobotObstacles(const FieldOptions& options) {
  std::vector<Box> boxes;
  AddHubs(boxes);
  AddTowers(boxes);
  AddTrenchSupports(boxes);
  if (!options.robotCanCrossBumps) {
    AddBumps(boxes);
  }
  if (!options.robotFitsUnderTrench) {
    AddTrenches(boxes);
  }
  return boxes;
}

std::vector<Box> BallObstacles() {
  std::vector<Box> boxes;
  AddHubs(boxes);
  AddTowers(boxes);
  AddTrenchSupports(boxes);
  return boxes;
}

double FloorHeightAt(const frc::Translation2d& point) {
  const double x = point.X().value();
  const double y = point.Y().value();
  for (const frc::Translation2d& hub : {constants::kBlueHubCenter, constants::kRedHubCenter}) {
    const double fromCenter = std::abs(x - hub.X().value());
    if (fromCenter >= constants::kBumpHalfDepthMeters) {
      continue;
    }
    const double hubMinY = hub.Y().value() - constants::kHubHalfSizeMeters;
    const double hubMaxY = hub.Y().value() + constants::kHubHalfSizeMeters;
    const bool onLowBump = y >= hubMinY - constants::kBumpLengthMeters && y <= hubMinY;
    const bool onHighBump = y >= hubMaxY && y <= hubMaxY + constants::kBumpLengthMeters;
    if (onLowBump || onHighBump) {
      return BumpProfile(fromCenter);
    }
  }
  return 0.0;
}

std::vector<frc::Translation2d> StartingFieldBalls() {
  std::vector<frc::Translation2d> balls;
  const frc::Translation2d fieldCenter{units::meter_t{constants::kFieldLengthMeters / 2.0},
                                       units::meter_t{constants::kFieldWidthMeters / 2.0}};
  AddGrid(balls, fieldCenter, constants::kNeutralBallColumns, constants::kNeutralBallRows,
          constants::kBallGridSpacingMeters);
  AddGrid(balls, constants::kBlueDepotCenter, constants::kDepotBallColumns, constants::kDepotBallRows,
          constants::kBallGridSpacingMeters);
  AddGrid(balls, constants::kRedDepotCenter, constants::kDepotBallColumns, constants::kDepotBallRows,
          constants::kBallGridSpacingMeters);
  return balls;
}

Hub GoalFor(bool isRed) {
  const frc::Translation2d center = isRed ? constants::kRedHubCenter : constants::kBlueHubCenter;
  // Scored balls come out on the neutral-zone side (toward the middle of the field)
  const double towardMiddle = center.X().value() < constants::kFieldLengthMeters / 2.0 ? 1.0 : -1.0;
  const frc::Translation2d exit{center.X() + units::meter_t{towardMiddle * constants::kHubExitDistanceMeters},
                                center.Y()};
  return Hub{center, constants::kHubOpeningHeightMeters, constants::kHubOpeningRadiusMeters,
             constants::kHubOuterRadiusMeters, exit, constants::kHubExitSpeedMps};
}

}  // namespace robotsim

#endif  // SIM_SEASON_YEAR == 2026
