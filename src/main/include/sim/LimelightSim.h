#pragma once

// ================================================================
// Fake Limelights. Publishes to the same NetworkTables entries real
// Limelights use ("limelight", "limelight-two"), so VisionSubsystem and
// DriveSubsystem's MegaTag code run with no changes.
//
// Dashboard controls (SmartDashboard):
//   Sim/Vision/<camera>/Enabled   - false = camera unplugged / blocked
//   Sim/Vision/NoiseScale         - 0 = perfect, 1 = normal, 3 = bad lighting
//   Sim/Vision/ExtraLatencyMs     - add network lag
// ================================================================

#include <random>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructArrayTopic.h>

#include "sim/LimelightModel.h"
#include "sim/SimConstants.h"

namespace robotsim {

class LimelightSim {
 public:
  LimelightSim();

  // Call every loop with the robot's TRUE pose.
  void Update(const frc::Pose2d& truePose);

 private:
  struct Camera {
    constants::CameraConfig config;
    nt::DoubleArrayPublisher megaTag1;
    nt::DoubleArrayPublisher megaTag2;
    nt::DoubleArrayPublisher targetSpace;
    nt::DoublePublisher tv;
    nt::DoublePublisher tx;
    nt::DoublePublisher ty;
    nt::DoublePublisher ta;
    nt::DoublePublisher tid;
    nt::DoublePublisher tl;
    nt::DoublePublisher cl;
    nt::DoubleArraySubscriber robotOrientation;  // written by VisionSubsystem::SetRobotOrientation
    nt::StructArrayPublisher<frc::Pose3d> visibleTags;  // for AdvantageScope
    CameraNoise noise;  // this camera's slowly-drifting measurement error
  };

  void PublishFrame(Camera& camera, double nowSeconds, double dtSeconds, const frc::Pose2d& truePose,
                    const NoiseSettings& noise, double extraLatencyMs);

  frc::AprilTagFieldLayout m_layout;
  frc::TimeInterpolatableBuffer<frc::Pose2d> m_history;
  std::vector<Camera> m_cameras;
  double m_lastUpdateSeconds = -1.0;
};

}  // namespace robotsim
