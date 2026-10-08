#pragma once

// ================================================================
// Simulation constants. Only used by the simulator (src/main/cpp/sim),
// never on the real robot. Split three ways so most of the sim carries
// over from year to year:
//
//   sim/season/SeasonYYYY.h  - the GAME: field, goals, structures, game piece
//   sim/robot/RobotConfig.h  - our ROBOT: size, gearing, mechanisms, cameras
//   this file                - general physics and sensor models (every year)
//
// New season: add sim/season/SeasonYYYY.h + src/main/cpp/sim/season/SeasonYYYY.cpp
// (copy the 2026 ones), then change SIM_SEASON_YEAR and the season #include below.
// New robot: change RobotConfig.h and sim/robot/RobotBinding.cpp.
// ================================================================

#include <numbers>

// Which season's field / game piece the sim uses (only that SeasonYYYY.cpp is compiled)
#define SIM_SEASON_YEAR 2026

#include "sim/robot/RobotConfig.h"
#include "sim/season/Season2026.h"

namespace robotsim::constants {

// ---------------- Physics ----------------
inline constexpr double kGravity = 9.81;
inline constexpr double kAirDensity = 1.2;
// Drag on the game piece: drag acceleration = kDragFactor * speed^2 (1/m)
inline constexpr double kDragFactor =
    0.5 * kAirDensity * kBallDragCoefficient * std::numbers::pi *
    (kBallDiameterMeters / 2) * (kBallDiameterMeters / 2) / kBallMassKg;

// ---------------- Limelight model ----------------
// Measurement noise at 1 m from one tag; it grows with distance^2 and shrinks with more tags.
inline constexpr double kVisionXYNoiseMeters = 0.02;
inline constexpr double kVisionHeadingNoiseDegMultiTag = 0.7;
inline constexpr double kVisionHeadingNoiseDegSingleTag = 4.0;
inline constexpr double kVisionLatencyJitterMs = 5.0;
// How the error changes over time: it wanders with this time constant, and only
// this fraction of it is new frame-to-frame jitter. TODO: fit to a real match log.
inline constexpr double kVisionNoiseDriftSeconds = 1.0;
inline constexpr double kVisionNoiseJitterFraction = 0.3;
inline constexpr unsigned kRandomSeed = 1229;  // fixed seed = repeatable runs

}  // namespace robotsim::constants
