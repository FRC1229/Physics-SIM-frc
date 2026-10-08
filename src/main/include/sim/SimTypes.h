#pragma once

// ================================================================
// Small shared types for the sim's season / robot settings files.
// ================================================================

#include <string_view>

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace robotsim::constants {

// One Limelight on the robot.
struct CameraConfig {
  std::string_view name;          // NetworkTables name (same as the robot code uses)
  frc::Transform3d robotToCamera;  // where the camera sits on the robot
  double horizontalFovDeg;
  double verticalFovDeg;
  double maxRangeMeters;          // tags farther than this are not detected
  double maxViewAngleDeg;         // tags seen too edge-on are not detected
  double latencyMs;               // capture + pipeline latency
};

// Camera mount: forward/left/up from robot center (m), pitch up and yaw (deg).
// WPILib's pitch is positive DOWN, so "pitch up" is negated here.
inline frc::Transform3d CameraMount(double forward, double left, double up, double pitchUpDeg,
                                    double yawDeg) {
  return frc::Transform3d{
      frc::Translation3d{units::meter_t{forward}, units::meter_t{left}, units::meter_t{up}},
      frc::Rotation3d{0_deg, units::degree_t{-pitchUpDeg}, units::degree_t{yawDeg}}};
}

}  // namespace robotsim::constants
