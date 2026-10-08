#include <cmath>
#include <random>

#include <frc/apriltag/AprilTagFields.h>
#include <gtest/gtest.h>

#include "sim/LimelightModel.h"
#include "sim/SimConstants.h"
#include "subsystems/VisionSubsystem.h"

using namespace robotsim;

namespace {

const frc::AprilTagFieldLayout& Layout() {
  static const frc::AprilTagFieldLayout layout =
      frc::AprilTagFieldLayout::LoadField(constants::kAprilTagField);
  return layout;
}

constexpr double kLatencyMs = 30.0;

NoiseSettings NoNoise() {
  NoiseSettings noise;
  noise.scale = 0.0;
  return noise;
}

const constants::CameraConfig& FrontCamera() { return constants::kCameras[0]; }

}  // namespace

TEST(LimelightModelTest, FrontCameraSeesHubTagsFromStartPose) {
  const std::vector<TagSighting> tags = VisibleTags(constants::kStartPose, FrontCamera(), Layout());
  ASSERT_FALSE(tags.empty());
  for (size_t i = 0; i < tags.size(); ++i) {
    EXPECT_LE(std::abs(tags[i].txDeg), FrontCamera().horizontalFovDeg / 2.0);
    EXPECT_LE(tags[i].distanceToCamera, FrontCamera().maxRangeMeters);
    EXPECT_GT(tags[i].robotToTag.X().value(), 0.0) << "front camera saw a tag behind the robot";
    if (i > 0) {
      EXPECT_GE(tags[i].distanceToCamera, tags[i - 1].distanceToCamera) << "not sorted closest first";
    }
  }
}

TEST(LimelightModelTest, TurningAroundChangesWhatTheCameraSees) {
  const frc::Pose2d facingAway{constants::kStartPose.Translation(), frc::Rotation2d{180_deg}};
  for (const TagSighting& tag : VisibleTags(facingAway, FrontCamera(), Layout())) {
    EXPECT_GT(tag.robotToTag.X().value(), 0.0);
  }
}

TEST(LimelightModelTest, PerfectFrameMatchesTruePose) {
  CameraNoise rng{1};
  const frc::Pose2d truth{2.7_m, 3.6_m, frc::Rotation2d{15_deg}};
  const std::vector<TagSighting> tags = VisibleTags(truth, FrontCamera(), Layout());
  ASSERT_FALSE(tags.empty());

  const LimelightFrame frame = BuildFrame(truth, truth.Rotation(), tags, kLatencyMs, NoNoise(), rng);
  ASSERT_TRUE(frame.hasTarget);
  EXPECT_NEAR(frame.megaTag1[0], 2.7, 1e-6);
  EXPECT_NEAR(frame.megaTag1[1], 3.6, 1e-6);
  EXPECT_NEAR(frame.megaTag1[5], 15.0, 1e-6);
  EXPECT_NEAR(frame.megaTag2[0], 2.7, 1e-6);
  EXPECT_NEAR(frame.megaTag2[1], 3.6, 1e-6);
  EXPECT_EQ(frame.tid, tags.front().id);
}

TEST(LimelightModelTest, MegaTag2WithWrongHeadingGivesWrongPosition) {
  CameraNoise rng{1};
  const frc::Pose2d truth = constants::kStartPose;
  const std::vector<TagSighting> tags = VisibleTags(truth, FrontCamera(), Layout());
  ASSERT_FALSE(tags.empty());

  const frc::Rotation2d wrongHeading = truth.Rotation() + frc::Rotation2d{10_deg};
  const LimelightFrame frame = BuildFrame(truth, wrongHeading, tags, kLatencyMs, NoNoise(), rng);
  const double error = std::hypot(frame.megaTag2[0] - truth.X().value(), frame.megaTag2[1] - truth.Y().value());
  EXPECT_GT(error, 0.1);
  // MegaTag1 solves heading itself, so it is still right
  EXPECT_NEAR(frame.megaTag1[0], truth.X().value(), 1e-6);
}

TEST(LimelightModelTest, NoTagsPublishesZerosLikeARealLimelight) {
  CameraNoise rng{1};
  const LimelightFrame frame = BuildFrame(constants::kStartPose, frc::Rotation2d{}, {}, kLatencyMs,
                                          NoNoise(), rng);
  EXPECT_FALSE(frame.hasTarget);
  ASSERT_EQ(frame.megaTag2.size(), 11u);
  EXPECT_EQ(frame.megaTag2[7], 0.0);
}

// The robot's own parser and filter (VisionSubsystem.cpp) must accept the fake data.
TEST(LimelightModelTest, RobotVisionCodeAcceptsSimulatedFrames) {
  CameraNoise rng{1};
  const frc::Pose2d truth = constants::kStartPose;
  const std::vector<TagSighting> tags = VisibleTags(truth, FrontCamera(), Layout());
  ASSERT_FALSE(tags.empty());
  const LimelightFrame frame = BuildFrame(truth, truth.Rotation(), tags, kLatencyMs, NoiseSettings{}, rng);

  const std::optional<vision::PoseObservation> parsed = vision::ParseBotpose(frame.megaTag2, 10_s);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->tagCount, static_cast<int>(tags.size()));
  EXPECT_NEAR(parsed->timestamp.value(), 10.0 - kLatencyMs / 1000.0, 1e-9);
  EXPECT_LT(parsed->pose.Translation().Distance(truth.Translation()).value(), 0.2);

  const vision::FilterConfig config{};
  EXPECT_EQ(vision::CheckMegaTag2(parsed.value(), 0.0, std::nullopt, truth.Rotation(), config),
            vision::RejectReason::kAccepted);
}

// A real camera's error wanders slowly; it does not jump every frame.
// Overall size stays the same (sigma), but frame-to-frame changes are much smaller.
TEST(LimelightModelTest, CameraNoiseDriftsInsteadOfJumping) {
  CameraNoise noise{7};
  constexpr double kSigma = 0.1;
  constexpr int kFrames = 20000;
  double previous = noise.Sample(NoiseChannel::kMegaTag2X, kSigma);
  double sumSquares = 0.0;
  double sumStepSquares = 0.0;
  for (int i = 0; i < kFrames; ++i) {
    noise.Step(0.02);
    const double sample = noise.Sample(NoiseChannel::kMegaTag2X, kSigma);
    sumSquares += sample * sample;
    sumStepSquares += (sample - previous) * (sample - previous);
    previous = sample;
  }
  const double spread = std::sqrt(sumSquares / kFrames);
  const double stepSpread = std::sqrt(sumStepSquares / kFrames);
  EXPECT_NEAR(spread, kSigma, 0.25 * kSigma) << "overall error size should not change";
  // Independent frames would give sqrt(2) * sigma per step
  EXPECT_LT(stepSpread, 0.6 * kSigma) << "error should change smoothly between frames";
  EXPECT_EQ(noise.Sample(NoiseChannel::kMegaTag2X, 0.0), 0.0);
}
