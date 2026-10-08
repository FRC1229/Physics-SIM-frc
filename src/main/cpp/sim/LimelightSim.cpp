#include "sim/LimelightSim.h"

#include <string>

#include <frc/Timer.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

namespace robotsim {

namespace {

constexpr units::second_t kPoseHistory = 1.5_s;
constexpr double kCaptureLatencyMs = 11.0;  // Limelight "cl"; the rest of the latency is "tl"
constexpr std::string_view kNoiseScaleKey = "Sim/Vision/NoiseScale";
constexpr std::string_view kExtraLatencyKey = "Sim/Vision/ExtraLatencyMs";

std::string EnabledKey(std::string_view cameraName) {
  return "Sim/Vision/" + std::string{cameraName} + "/Enabled";
}

}  // namespace

LimelightSim::LimelightSim()
    : m_layout(frc::AprilTagFieldLayout::LoadField(constants::kAprilTagField)),
      m_history(kPoseHistory) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  unsigned seed = constants::kRandomSeed;  // fixed seeds = repeatable runs
  for (const constants::CameraConfig& config : constants::kCameras) {
    auto table = inst.GetTable(config.name);
    m_cameras.push_back(Camera{
        config,
        table->GetDoubleArrayTopic("botpose_wpiblue").Publish(),
        table->GetDoubleArrayTopic("botpose_orb_wpiblue").Publish(),
        table->GetDoubleArrayTopic("botpose_targetspace").Publish(),
        table->GetDoubleTopic("tv").Publish(),
        table->GetDoubleTopic("tx").Publish(),
        table->GetDoubleTopic("ty").Publish(),
        table->GetDoubleTopic("ta").Publish(),
        table->GetDoubleTopic("tid").Publish(),
        table->GetDoubleTopic("tl").Publish(),
        table->GetDoubleTopic("cl").Publish(),
        table->GetDoubleArrayTopic("robot_orientation_set").Subscribe({}),
        inst.GetStructArrayTopic<frc::Pose3d>("Sim/Vision/" + std::string{config.name} + "/VisibleTags")
            .Publish(),
        CameraNoise{seed++}});
    frc::SmartDashboard::SetDefaultBoolean(EnabledKey(config.name), true);
  }
  frc::SmartDashboard::SetDefaultNumber(kNoiseScaleKey, 1.0);
  frc::SmartDashboard::SetDefaultNumber(kExtraLatencyKey, 0.0);
}

void LimelightSim::Update(const frc::Pose2d& truePose) {
  const units::second_t now = frc::Timer::GetFPGATimestamp();
  m_history.AddSample(now, truePose);

  NoiseSettings noise;
  noise.scale = std::max(0.0, frc::SmartDashboard::GetNumber(kNoiseScaleKey, 1.0));
  const double extraLatencyMs = std::max(0.0, frc::SmartDashboard::GetNumber(kExtraLatencyKey, 0.0));
  const double dtSeconds = m_lastUpdateSeconds < 0.0 ? 0.0 : now.value() - m_lastUpdateSeconds;
  m_lastUpdateSeconds = now.value();

  for (Camera& camera : m_cameras) {
    if (!frc::SmartDashboard::GetBoolean(EnabledKey(camera.config.name), true)) {
      camera.visibleTags.Set({});
      continue;  // unplugged: a real Limelight stops sending new frames
    }
    PublishFrame(camera, now.value(), dtSeconds, truePose, noise, extraLatencyMs);
  }
}

void LimelightSim::PublishFrame(Camera& camera, double nowSeconds, double dtSeconds,
                                const frc::Pose2d& truePose, const NoiseSettings& noise,
                                double extraLatencyMs) {
  camera.noise.Step(dtSeconds);
  std::uniform_real_distribution<double> jitter{-constants::kVisionLatencyJitterMs,
                                                constants::kVisionLatencyJitterMs};
  const double latencyMs = camera.config.latencyMs + jitter(camera.noise.Rng()) + extraLatencyMs;

  // The image was taken latencyMs ago, so measure where the robot was then.
  const units::second_t captureTime{nowSeconds - latencyMs / 1000.0};
  const frc::Pose2d capturePose = m_history.Sample(captureTime).value_or(truePose);

  const std::vector<double> orientation = camera.robotOrientation.Get();
  const frc::Rotation2d sentHeading{units::degree_t{orientation.empty() ? 0.0 : orientation[0]}};

  const std::vector<TagSighting> tags = VisibleTags(capturePose, camera.config, m_layout);
  const LimelightFrame frame = BuildFrame(capturePose, sentHeading, tags, latencyMs, noise, camera.noise);

  camera.megaTag1.Set(frame.megaTag1);
  camera.megaTag2.Set(frame.megaTag2);
  camera.targetSpace.Set(frame.targetSpace);
  camera.tv.Set(frame.hasTarget ? 1.0 : 0.0);
  camera.tx.Set(frame.tx);
  camera.ty.Set(frame.ty);
  camera.ta.Set(frame.ta);
  camera.tid.Set(frame.tid);
  camera.tl.Set(latencyMs - kCaptureLatencyMs);
  camera.cl.Set(kCaptureLatencyMs);

  std::vector<frc::Pose3d> tagPoses;
  tagPoses.reserve(tags.size());
  for (const TagSighting& tag : tags) {
    tagPoses.push_back(tag.tagPose);
  }
  camera.visibleTags.Set(tagPoses);
}

}  // namespace robotsim
