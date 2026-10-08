#pragma once

// ================================================================
// Fake Limelight math (pure, no NetworkTables).
// Works out which AprilTags a camera can see from a robot pose, and builds
// the same botpose arrays a real Limelight publishes, so the robot's real
// vision code (VisionSubsystem / vision::ParseBotpose) reads them unchanged.
//
// botpose array layout (Limelight docs):
//   [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea,
//    then 7 values per tag: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
// ================================================================

#include <array>
#include <random>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

#include "sim/SimConstants.h"

namespace robotsim {

struct TagSighting {
  int id = 0;
  frc::Pose3d tagPose;              // field pose of the tag
  frc::Translation2d robotToTag;    // tag position in the ROBOT frame (x forward, y left)
  double distanceToCamera = 0.0;    // meters
  double distanceToRobot = 0.0;     // meters
  double txDeg = 0.0;               // Limelight tx: + = tag is right of crosshair
  double tyDeg = 0.0;               // Limelight ty: + = tag is above crosshair
  double areaPercent = 0.0;         // Limelight ta: % of the image
};

// Tags the camera can see from this robot pose (in view, in range, not too edge-on).
// Sorted closest first (index 0 = the Limelight's "primary" tag).
std::vector<TagSighting> VisibleTags(const frc::Pose2d& robotPose,
                                     const constants::CameraConfig& camera,
                                     const frc::AprilTagFieldLayout& layout);

struct NoiseSettings {
  double scale = 1.0;  // 0 = perfect measurements
  double xyMetersAtOneMeter = constants::kVisionXYNoiseMeters;
  double headingDegMultiTag = constants::kVisionHeadingNoiseDegMultiTag;
  double headingDegSingleTag = constants::kVisionHeadingNoiseDegSingleTag;
};

// One camera's measurement error. A real Limelight's error does NOT jump to a
// new random value every frame: the image barely changes between frames, so the
// error wanders slowly, with only a little frame-to-frame jitter on top.
// Each sample has the requested standard deviation overall; most of it is the
// slow drift (time constant kVisionNoiseDriftSeconds), the rest is jitter.
enum class NoiseChannel { kMegaTag1X, kMegaTag1Y, kMegaTag1Heading, kMegaTag2X, kMegaTag2Y, kCount };

class CameraNoise {
 public:
  explicit CameraNoise(unsigned seed);

  // Lets the slow drift move on by dtSeconds (call once per camera frame).
  void Step(double dtSeconds);

  // A random error with standard deviation `sigma` on one channel.
  double Sample(NoiseChannel channel, double sigma);

  // The rest of the camera's randomness (latency jitter) comes from here too.
  std::mt19937& Rng() { return m_rng; }

 private:
  static constexpr size_t kChannels = static_cast<size_t>(NoiseChannel::kCount);
  std::mt19937 m_rng;
  std::array<double, kChannels> m_drift{};  // each one ~ N(0, 1), changing slowly
};

struct LimelightFrame {
  std::vector<double> megaTag1;      // botpose_wpiblue
  std::vector<double> megaTag2;      // botpose_orb_wpiblue
  std::vector<double> targetSpace;   // botpose_targetspace (robot pose relative to primary tag)
  bool hasTarget = false;
  double tx = 0.0;
  double ty = 0.0;
  double ta = 0.0;
  double tid = -1.0;
};

// Builds one frame.
//   truePose: where the robot really was when the image was captured
//   sentHeading: the heading the robot code sent in robot_orientation_set (used by MegaTag2,
//                exactly like a real Limelight, so a wrong heading gives a wrong MegaTag2 X/Y)
//   latencyMs: written into the arrays (the robot code subtracts it from the receive time)
LimelightFrame BuildFrame(const frc::Pose2d& truePose, const frc::Rotation2d& sentHeading,
                          const std::vector<TagSighting>& tags, double latencyMs,
                          const NoiseSettings& noise, CameraNoise& cameraNoise);

}  // namespace robotsim
