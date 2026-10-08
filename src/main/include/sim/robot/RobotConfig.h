#pragma once

// ================================================================
// ROBOT: our 2026 robot. Everything about the MACHINE (not the game):
// size, mass, gearing, mechanism limits, camera mounts.
//
// Values marked "TODO: measure" are estimates. The sim works with them,
// but it gets more accurate when you replace them with real numbers.
// New robot? Copy this file, change the values, and point sim/SimConstants.h at it.
// ================================================================

#include <array>

#include <units/mass.h>
#include <units/moment_of_inertia.h>

#include "sim/SimTypes.h"

namespace robotsim::constants {

// ---------------- Robot ----------------
inline constexpr units::kilogram_t kRobotMass = 60_kg;  // TODO: measure (with bumpers + battery)
inline constexpr double kBumperLengthMeters = 0.85;      // TODO: measure (front to back)
inline constexpr double kBumperWidthMeters = 0.80;       // TODO: measure (side to side)
// The robot drives over the bumps and under the trenches (team confirmed).
// Flip on the dashboard (Sim/Field/...) to test what happens if it couldn't.
inline constexpr bool kRobotFitsUnderTrench = true;
inline constexpr bool kRobotCanCrossBumps = true;

// ---------------- Swerve ----------------
// Drive gearing and wheel radius match ModuleConstants::velocityScaleFactor.
inline constexpr double kDriveGearing = 6.75;
inline constexpr double kWheelRadiusMeters = 0.0508;
inline constexpr double kSteerGearing = 150.0 / 7.0;  // TODO: measure (150/7 = MK4i, 12.8 = MK4)
inline constexpr units::kilogram_square_meter_t kSteerInertia = 0.004_kg_sq_m;

// ---------------- Shooter ----------------
// Hood angle MEASURED on the robot (60 deg). The exit speed is fitted so the
// team's ShooterConstants::sotm_rpms table scores from 2-5 m (one factor fits
// every distance within +-2%). Note: with a 60 deg hood the ball is in the air
// ~0.24 s LESS than the measured tof_seconds at every distance, so those times
// most likely include the ball's trip through the kicker (see kBallTransitSeconds).
inline constexpr double kHoodAngleDeg = 60.0;             // measured
inline constexpr double kExitSpeedPerRpm = 0.00425;       // m/s of ball speed per flywheel RPM (fitted)
// Time from the feeder pushing a ball into the kicker until it leaves the shooter.
// = measured tof_seconds minus the 60 deg air time (the gap is ~0.24 s at every
// distance). TODO: measure with slow-motion video (feed starts -> ball exits).
inline constexpr double kBallTransitSeconds = 0.24;
inline constexpr double kLaunchHeightMeters = 0.50;       // TODO: measure (ball exit height)
inline constexpr double kFlywheelGearing = 1.0;           // motor : flywheel
inline constexpr units::kilogram_square_meter_t kFlywheelInertia = 0.003_kg_sq_m;  // TODO: measure
inline constexpr double kFlywheelSpeedLossPerShot = 0.05;  // fraction of speed each ball takes
// Real flywheel friction, from the team's MEASURED feedforward (ShooterSubsystem
// m_feedforward: kS = 0.2 V, kV = 0.00197676 V per motor RPM). An ideal Vortex
// only needs 12 V / 6784 RPM = 0.00177 V/RPM, so the extra voltage is friction.
// Without it the sim flywheel spins ~6% faster than the real one for the same voltage.
// TODO: replace with SysId results when you have them.
inline constexpr double kFlywheelFrictionVolts = 0.2;            // kS
inline constexpr double kFlywheelMeasuredVoltsPerRpm = 0.00197676; // kV
inline constexpr double kBallsPerSecond = 5.0;            // TODO: measure feed rate
inline constexpr double kMinFeedOutput = 0.1;             // indexer output that counts as "feeding"
inline constexpr int kHopperCapacity = 40;                // TODO: measure (most balls the robot can hold)

// ---------------- Intake ----------------
// IntakeSubsystem::GetPosition() = motorRotations * 360 / 42, so with a 42:1
// gearbox it reads true arm degrees (0 = stowed, 190 = kLowered).
inline constexpr double kIntakeGearing = 42.0;             // TODO: measure
inline constexpr units::kilogram_square_meter_t kIntakeArmInertia = 0.15_kg_sq_m;  // TODO: measure
inline constexpr double kIntakeMinDeg = 0.0;               // hard stop, stowed
inline constexpr double kIntakeMaxDeg = 195.0;             // hard stop, on the carpet. TODO: measure
inline constexpr double kIntakeDeployedDeg = 150.0;        // arm low enough to reach balls. TODO: measure
inline constexpr double kMinRollerOutput = 0.1;            // roller output that counts as "intaking"
inline constexpr double kIntakeBallsPerSecond = 8.0;       // TODO: measure
// Which end the intake is on (robot frame +x = front). The team confirmed it is
// on the FRONT.
inline constexpr bool kIntakeAtBack = false;
// Grab area, robot frame: from the intake-side bumper out to the roller's reach. TODO: measure
inline constexpr double kIntakeReachMeters = 0.30;
inline constexpr double kIntakeHalfWidthMeters = 0.30;
// Arm pivot on the robot (for the AdvantageScope component). TODO: measure
inline constexpr double kIntakePivotForwardMeters = 0.30;
inline constexpr double kIntakePivotUpMeters = 0.25;

// ---------------- Turret ----------------
// TurretSubsystem::readEncoder() uses motorRotations * 0.75 = turret degrees,
// so 1 turret turn = 480 motor rotations.
inline constexpr double kTurretDegreesPerMotorRotation = 0.75;
inline constexpr double kTurretGearing = 360.0 / kTurretDegreesPerMotorRotation;
inline constexpr units::kilogram_square_meter_t kTurretInertia = 0.05_kg_sq_m;  // TODO: measure
// Physical hard stops, robot-relative degrees (0 = robot forward).
// The code's soft limits are readEncoder() 65..270 = -115..+90.
inline constexpr double kTurretMinDeg = -150.0;  // TODO: measure
inline constexpr double kTurretMaxDeg = 150.0;   // TODO: measure

// ---------------- Limelights ----------------
// TODO: measure. Copy these from each Limelight's web UI ("Robot Space" settings).
// Limelight 3: 62.5 x 48.9 deg FOV.
inline const std::array<CameraConfig, 2> kCameras{{
    {"limelight", CameraMount(0.30, 0.0, 0.50, 20.0, 0.0), 62.5, 48.9, 6.5, 70.0, 30.0},
    {"limelight-two", CameraMount(-0.30, 0.0, 0.50, 20.0, 180.0), 62.5, 48.9, 6.5, 70.0, 30.0},
}};

}  // namespace robotsim::constants
