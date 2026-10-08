#include "sim/LimelightModel.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace robotsim {

namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kTagSizeMeters = 0.1651;  // 6.5 in AprilTag
constexpr size_t kBaseArrayLength = 11;
constexpr size_t kTargetSpaceLength = 6;
constexpr double kSingleTagAmbiguity = 0.05;

double Gaussian(std::mt19937& rng, double sigma) {
  if (sigma <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> distribution{0.0, sigma};
  return distribution(rng);
}

// Largest distance between any two seen tags (Limelight "tag span").
double TagSpan(const std::vector<TagSighting>& tags) {
  double span = 0.0;
  for (size_t i = 0; i < tags.size(); ++i) {
    for (size_t j = i + 1; j < tags.size(); ++j) {
      const double d = tags[i].tagPose.Translation().Distance(tags[j].tagPose.Translation()).value();
      span = std::max(span, d);
    }
  }
  return span;
}

std::vector<double> PoseArray(const frc::Pose2d& pose, double latencyMs,
                              const std::vector<TagSighting>& tags) {
  double distanceSum = 0.0;
  double areaSum = 0.0;
  for (const TagSighting& tag : tags) {
    distanceSum += tag.distanceToCamera;
    areaSum += tag.areaPercent;
  }
  const double count = static_cast<double>(tags.size());

  std::vector<double> data{pose.X().value(), pose.Y().value(), 0.0, 0.0, 0.0,
                           pose.Rotation().Degrees().value(), latencyMs, count,
                           TagSpan(tags), distanceSum / count, areaSum / count};
  for (const TagSighting& tag : tags) {
    data.insert(data.end(), {static_cast<double>(tag.id), tag.txDeg, tag.tyDeg, tag.areaPercent,
                             tag.distanceToCamera, tag.distanceToRobot,
                             tags.size() == 1 ? kSingleTagAmbiguity : 0.0});
  }
  return data;
}

// MegaTag1: the camera solves the full pose itself (heading included).
frc::Pose2d MegaTag1Pose(const frc::Pose2d& truePose, const std::vector<TagSighting>& tags,
                         double averageDistance, const NoiseSettings& noise, CameraNoise& cameraNoise) {
  const double count = static_cast<double>(tags.size());
  const double xySigma = noise.scale * noise.xyMetersAtOneMeter * averageDistance * averageDistance / count;
  const double headingBase = tags.size() >= 2 ? noise.headingDegMultiTag : noise.headingDegSingleTag;
  const double headingSigma = noise.scale * headingBase * averageDistance / std::sqrt(count);
  return frc::Pose2d{
      truePose.X() + units::meter_t{cameraNoise.Sample(NoiseChannel::kMegaTag1X, xySigma)},
      truePose.Y() + units::meter_t{cameraNoise.Sample(NoiseChannel::kMegaTag1Y, xySigma)},
      truePose.Rotation() +
          frc::Rotation2d{units::degree_t{cameraNoise.Sample(NoiseChannel::kMegaTag1Heading, headingSigma)}}};
}

// MegaTag2: the camera trusts the heading the robot sent and only solves X/Y.
// Each tag gives "tag position minus (what I see, rotated by the sent heading)";
// the tags are averaged, so the error shrinks with sqrt(tag count).
frc::Pose2d MegaTag2Pose(const frc::Rotation2d& sentHeading, const std::vector<TagSighting>& tags,
                         const NoiseSettings& noise, CameraNoise& cameraNoise) {
  double sumX = 0.0;
  double sumY = 0.0;
  double sumDistanceSquared = 0.0;
  for (const TagSighting& tag : tags) {
    const frc::Translation2d robot =
        tag.tagPose.ToPose2d().Translation() - tag.robotToTag.RotateBy(sentHeading);
    sumX += robot.X().value();
    sumY += robot.Y().value();
    sumDistanceSquared += tag.distanceToCamera * tag.distanceToCamera;
  }
  const double count = static_cast<double>(tags.size());
  const double sigma =
      0.5 * noise.scale * noise.xyMetersAtOneMeter * (sumDistanceSquared / count) / std::sqrt(count);
  return frc::Pose2d{units::meter_t{sumX / count + cameraNoise.Sample(NoiseChannel::kMegaTag2X, sigma)},
                     units::meter_t{sumY / count + cameraNoise.Sample(NoiseChannel::kMegaTag2Y, sigma)},
                     sentHeading};
}

// Robot pose in the primary tag's frame, Limelight target-space convention
// (x right, y down, z out of the tag).
std::vector<double> TargetSpace(const frc::Pose2d& truePose, const TagSighting& primary) {
  const frc::Pose2d inTagFrame = truePose.RelativeTo(primary.tagPose.ToPose2d());
  return {-inTagFrame.Y().value(), 0.0, inTagFrame.X().value(), 0.0,
          inTagFrame.Rotation().Degrees().value(), 0.0};
}

}  // namespace

CameraNoise::CameraNoise(unsigned seed) : m_rng(seed) {
  for (double& drift : m_drift) {
    drift = Gaussian(m_rng, 1.0);  // start "already wandering", not at zero
  }
}

void CameraNoise::Step(double dtSeconds) {
  // First-order random walk that always settles back toward zero (Gauss-Markov):
  // keeps each channel's spread at 1 while changing on the drift time scale.
  const double keep = std::exp(-std::max(dtSeconds, 0.0) / constants::kVisionNoiseDriftSeconds);
  const double fresh = std::sqrt(1.0 - keep * keep);
  for (double& drift : m_drift) {
    drift = keep * drift + fresh * Gaussian(m_rng, 1.0);
  }
}

double CameraNoise::Sample(NoiseChannel channel, double sigma) {
  if (sigma <= 0.0) {
    return 0.0;
  }
  const double jitter = constants::kVisionNoiseJitterFraction;
  const double drift = m_drift[static_cast<size_t>(channel)];
  // Drift and jitter are independent, so this keeps the overall spread at sigma
  return sigma * (std::sqrt(1.0 - jitter * jitter) * drift + jitter * Gaussian(m_rng, 1.0));
}

std::vector<TagSighting> VisibleTags(const frc::Pose2d& robotPose,
                                     const constants::CameraConfig& camera,
                                     const frc::AprilTagFieldLayout& layout) {
  const frc::Pose3d cameraPose = frc::Pose3d{robotPose}.TransformBy(camera.robotToCamera);
  const double halfH = camera.horizontalFovDeg / 2.0;
  const double halfV = camera.verticalFovDeg / 2.0;

  std::vector<TagSighting> seen;
  for (const frc::AprilTag& tag : layout.GetTags()) {
    const frc::Translation3d inCamera = tag.pose.RelativeTo(cameraPose).Translation();
    const double x = inCamera.X().value();
    const double y = inCamera.Y().value();
    const double z = inCamera.Z().value();
    if (x <= 0.0) {
      continue;  // behind the camera
    }
    const double yawDeg = std::atan2(y, x) * kRadToDeg;
    const double pitchDeg = std::atan2(z, std::hypot(x, y)) * kRadToDeg;
    const double distance = inCamera.Norm().value();
    if (std::abs(yawDeg) > halfH || std::abs(pitchDeg) > halfV || distance > camera.maxRangeMeters) {
      continue;
    }

    // The tag faces along its own +X axis; the camera must be in front of it.
    const frc::Translation3d cameraInTag = cameraPose.RelativeTo(tag.pose).Translation();
    const double facing = cameraInTag.X().value() / cameraInTag.Norm().value();
    const double viewAngleDeg = std::acos(std::clamp(facing, -1.0, 1.0)) * kRadToDeg;
    if (facing <= 0.0 || viewAngleDeg > camera.maxViewAngleDeg) {
      continue;
    }

    const double frameW = 2.0 * distance * std::tan(halfH * kDegToRad);
    const double frameH = 2.0 * distance * std::tan(halfV * kDegToRad);
    const frc::Translation2d tagOnField = tag.pose.ToPose2d().Translation();

    TagSighting sighting;
    sighting.id = tag.ID;
    sighting.tagPose = tag.pose;
    sighting.robotToTag = (tagOnField - robotPose.Translation()).RotateBy(-robotPose.Rotation());
    sighting.distanceToCamera = distance;
    sighting.distanceToRobot = robotPose.Translation().Distance(tagOnField).value();
    sighting.txDeg = -yawDeg;
    sighting.tyDeg = pitchDeg;
    sighting.areaPercent = 100.0 * kTagSizeMeters * kTagSizeMeters * facing / (frameW * frameH);
    seen.push_back(sighting);
  }

  std::sort(seen.begin(), seen.end(), [](const TagSighting& a, const TagSighting& b) {
    return a.distanceToCamera < b.distanceToCamera;
  });
  return seen;
}

LimelightFrame BuildFrame(const frc::Pose2d& truePose, const frc::Rotation2d& sentHeading,
                          const std::vector<TagSighting>& tags, double latencyMs,
                          const NoiseSettings& noise, CameraNoise& cameraNoise) {
  LimelightFrame frame;
  if (tags.empty()) {
    // A real Limelight still publishes, with zeros and tag count 0
    frame.megaTag1.assign(kBaseArrayLength, 0.0);
    frame.megaTag1[6] = latencyMs;
    frame.megaTag2 = frame.megaTag1;
    frame.targetSpace.assign(kTargetSpaceLength, 0.0);
    return frame;
  }

  double distanceSum = 0.0;
  for (const TagSighting& tag : tags) {
    distanceSum += tag.distanceToCamera;
  }
  const double averageDistance = std::max(distanceSum / tags.size(), 0.5);

  frame.megaTag1 = PoseArray(MegaTag1Pose(truePose, tags, averageDistance, noise, cameraNoise), latencyMs, tags);
  frame.megaTag2 = PoseArray(MegaTag2Pose(sentHeading, tags, noise, cameraNoise), latencyMs, tags);
  frame.targetSpace = TargetSpace(truePose, tags.front());
  frame.hasTarget = true;
  frame.tx = tags.front().txDeg;
  frame.ty = tags.front().tyDeg;
  frame.ta = tags.front().areaPercent;
  frame.tid = tags.front().id;
  return frame;
}

}  // namespace robotsim
