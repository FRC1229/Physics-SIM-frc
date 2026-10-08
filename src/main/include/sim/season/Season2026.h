#pragma once

// ================================================================
// SEASON: 2026 REBUILT. Everything about the GAME (not our robot):
// field size, AprilTags, goals, solid structures, the game piece, and
// where pieces start.
//
// New season? Copy this file to SeasonYYYY.h, change the values, and point
// sim/SimConstants.h at it. The shapes (hub, tower, bump, trench) are built
// in sim/FieldGeometry.cpp; a game with different structures needs that too.
// ================================================================

#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>

namespace robotsim::constants {

inline constexpr int kSeasonYear = 2026;
inline constexpr frc::AprilTagField kAprilTagField = frc::AprilTagField::k2026RebuiltWelded;

// ---------------- Field (blue-origin coordinates) ----------------
inline constexpr double kFieldLengthMeters = 16.541;
inline constexpr double kFieldWidthMeters = 8.069;

// Hub (goal) centers, from the official AprilTag layout (tags sit on the faces).
inline const frc::Translation2d kBlueHubCenter{4.626_m, 4.035_m};
inline const frc::Translation2d kRedHubCenter{11.915_m, 4.035_m};
inline constexpr double kHubOpeningHeightMeters = 1.829;  // 72 in
inline constexpr double kHubOpeningRadiusMeters = 0.53;   // hexagon opening, ~41.7 in across
inline constexpr double kHubOuterRadiusMeters = 0.60;     // hub body (balls hitting the side miss)

// Solid field structures (from the official 2026 AprilTag positions, which sit on
// their faces). Robots and rolling balls bounce off these.
inline constexpr double kHubHalfSizeMeters = 0.6035;  // tags at x 4.022 / 5.229: a 1.207 m square
// Towers: against each alliance wall, behind tags 31/32 (blue) and 15/16 (red).
inline constexpr double kTowerCenterYMeters = 3.962;
inline constexpr double kTowerDepthMeters = 1.15;      // TODO: verify against field drawings
inline constexpr double kTowerHalfWidthMeters = 0.50;  // TODO: verify against field drawings
// Bumps: the ramps between each hub and its trenches (73 in long). Robots can
// normally drive over them; Sim/Field/RobotCanCrossBumps = false makes them walls.
inline constexpr double kBumpLengthMeters = 1.854;     // along the field width
inline constexpr double kBumpHalfDepthMeters = 0.56;   // along the field length. TODO: verify
// Bump profile, across it: a ramp up, a flat top, a ramp down.
inline constexpr double kBumpHeightMeters = 0.165;     // ~6.5 in. TODO: verify against field drawings
inline constexpr double kBumpRampRunMeters = 0.45;     // each ramp, along the field length (~20 deg). TODO: verify
// Trenches: low arms from the side walls to the bumps (short robots drive under).
inline constexpr double kTrenchWidthMeters = 1.288;    // side wall out to the trench's inner edge
inline constexpr double kTrenchHalfDepthMeters = 0.15; // TODO: verify
// The trench's support between the end of each bump and the trench lane is solid
// (robots and balls hit it); it spans the bump's depth. TODO: verify against field drawings

// Where the robot starts when the sim launches or "Reset Robot To Start" is pressed.
// Blue side, inside the shooting zone (x < 4.5 m), facing away from the blue wall.
inline const frc::Pose2d kStartPose{2.5_m, 4.035_m, frc::Rotation2d{0_deg}};
inline constexpr int kStartingBalls = 8;  // preload. TODO: verify against the game manual

// ---------------- Game piece (FUEL) ----------------
inline constexpr double kBallMassKg = 0.215;
inline constexpr double kBallDiameterMeters = 0.150;
inline constexpr double kBallDragCoefficient = 0.47;

// Starting FUEL layout: a pile in the middle of the neutral zone plus one
// depot per alliance. TODO: verify counts and positions against the field drawings.
inline constexpr double kBallGridSpacingMeters = 0.16;
inline constexpr int kNeutralBallColumns = 12;  // along the field length
inline constexpr int kNeutralBallRows = 30;     // along the field width
inline constexpr int kDepotBallColumns = 4;
inline constexpr int kDepotBallRows = 6;
inline const frc::Translation2d kBlueDepotCenter{0.45_m, 5.95_m};
inline const frc::Translation2d kRedDepotCenter{16.09_m, 2.12_m};
// Scored balls come back out of the hub on the neutral-zone side, this far from its center.
inline constexpr bool kReturnScoredBalls = true;
inline constexpr double kHubExitDistanceMeters = 1.2;
inline constexpr double kHubExitSpeedMps = 1.5;          // TODO: measure (how fast scored balls roll out)

// Rolling ball physics (foam FUEL on carpet). TODO: tune against video
inline constexpr double kBallRollingDecelMps2 = 1.5;     // carpet friction: slows a rolling ball
inline constexpr double kBallWallRestitution = 0.35;     // speed kept bouncing off a wall / structure
inline constexpr double kBallBallRestitution = 0.5;      // speed kept in a ball-to-ball hit
inline constexpr double kBallLandingSpeedKept = 0.4;     // horizontal speed kept when a shot lands
inline constexpr double kBallStopSpeedMps = 0.03;        // slower than this = stopped

}  // namespace robotsim::constants
