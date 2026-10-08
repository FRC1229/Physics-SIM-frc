#include "sim/RobotSim.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>

#include "sim/SimConstants.h"
#include "sim/Telemetry.h"

namespace robotsim {

namespace {

// Test-drive settings (editable on the dashboard)
constexpr std::string_view kShuttleSpeedKey = "Sim/Test/ShuttleSpeedMps";
constexpr std::string_view kStrafeXKey = "Sim/Test/StrafeAtX";
constexpr double kDefaultShuttleSpeed = 1.5;
constexpr double kDefaultStrafeX = 2.5;
constexpr double kStrafeMinY = 2.5;
constexpr double kStrafeMaxY = 5.5;
constexpr double kTowardHubMinX = 1.5;
constexpr double kTowardHubMaxX = 3.4;
constexpr double kHoldGain = 2.0;     // m/s per meter off the line
constexpr double kHeadingGain = 3.0;  // rad/s per radian off heading 0
constexpr double kMaxHoldSpeed = 1.0;

// Which field structures the robot can get through (dashboard switches)
constexpr std::string_view kCrossBumpsKey = "Sim/Field/RobotCanCrossBumps";
constexpr std::string_view kUnderTrenchKey = "Sim/Field/RobotFitsUnderTrench";

// Shot testing: robot placed this far from the hub center, in this direction
// (field angle from the hub to the robot; 180 = straight out toward your alliance wall)
constexpr std::string_view kShotTestDistanceKey = "Sim/ShotTest/DistanceM";
constexpr std::string_view kShotTestAngleKey = "Sim/ShotTest/AngleDeg";
constexpr double kDefaultShotTestDistance = 3.0;
constexpr double kDefaultShotTestAngle = 150.0;  // 180 runs into the tower past ~3 m
constexpr std::array<double, 6> kSweepDistances{1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
constexpr int kSweepShotsPerSpot = 5;
constexpr units::second_t kSweepSpotTimeout = 8_s;
constexpr units::second_t kGyroSettleTime = 100_ms;
constexpr int kFieldBallPublishLoops = 5;  // 10 times a second
constexpr std::string_view kPlaceAtAutoStartKey = "Sim/Auto/PlaceAtStart";
constexpr int kGyroSettleLoops = 5;  // the simulated Pigeon reports a new heading a few loops later

}  // namespace

RobotSim::RobotSim(RobotHardware hardware, RobotCodeHooks hooks)
    : m_hooks(std::move(hooks)),
      m_drivetrain(std::move(hardware.drivetrain)),
      m_intake(std::move(hardware.intakeArm), std::move(hardware.intakeRoller)),
      m_shooter(std::move(hardware.shooter), m_fieldBalls) {
  m_fieldBalls.Reset(StartingFieldBalls());
  m_drivetrain.SetFieldOptions(m_fieldOptions);
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_truePosePublisher = inst.GetStructTopic<frc::Pose2d>("Sim/TruePose").Publish(EveryLoop());
  // With height and tilt (rises and tips going over a bump): use this for the 3D robot
  m_truePose3dPublisher = inst.GetStructTopic<frc::Pose3d>("Sim/TruePose3d").Publish(EveryLoop());
  m_fieldBallPublisher = inst.GetStructArrayTopic<frc::Pose3d>("Sim/FieldBalls").Publish();
  m_componentPublisher = inst.GetStructArrayTopic<frc::Pose3d>("Sim/Components").Publish(EveryLoop());
  m_poseErrorPublisher = inst.GetDoubleTopic("Sim/PoseErrorMeters").Publish(EveryLoop());
  m_headingErrorPublisher = inst.GetDoubleTopic("Sim/HeadingErrorDeg").Publish(EveryLoop());
  m_autoStartPublisher = inst.GetStructTopic<frc::Pose2d>("Sim/Auto/StartPose").Publish();
  frc::SmartDashboard::SetDefaultBoolean(kPlaceAtAutoStartKey, true);
  frc::SmartDashboard::SetDefaultNumber(kShuttleSpeedKey, kDefaultShuttleSpeed);
  frc::SmartDashboard::SetDefaultNumber(kStrafeXKey, kDefaultStrafeX);
  frc::SmartDashboard::SetDefaultBoolean(kCrossBumpsKey, m_fieldOptions.robotCanCrossBumps);
  frc::SmartDashboard::SetDefaultBoolean(kUnderTrenchKey, m_fieldOptions.robotFitsUnderTrench);
  frc::SmartDashboard::SetDefaultNumber(kShotTestDistanceKey, kDefaultShotTestDistance);
  frc::SmartDashboard::SetDefaultNumber(kShotTestAngleKey, kDefaultShotTestAngle);
  RegisterDashboardCommands();
}

void RobotSim::Update(units::second_t dt) {
  const auto loopStart = std::chrono::steady_clock::now();
  const units::volt_t battery = frc::RobotController::GetBatteryVoltage();

  UpdateFieldOptions();
  UpdateAutoPlacement();
  UpdateAutoReport();
  m_drivetrain.Update(dt, battery);
  const frc::Pose2d truePose = m_drivetrain.GetPose();
  m_limelights.Update(truePose);

  // Balls roll, bounce, and get shoved by the bumpers
  m_fieldBalls.Step(dt.value(), RobotBody{truePose, m_drivetrain.GetFieldRelativeSpeeds(),
                                          constants::kBumperLengthMeters / 2.0,
                                          constants::kBumperWidthMeters / 2.0});

  m_intake.Update(dt, battery);
  const int room = constants::kHopperCapacity - m_shooter.BallsInRobot();
  const int picked = m_intake.Collect(m_fieldBalls, truePose, room, dt);
  m_shooter.AddBalls(picked);
  m_ballsCollected += picked;

  m_shooter.Update(dt, battery, truePose, m_drivetrain.GetFieldRelativeSpeeds());
  // How long the simulator takes per loop (must stay well under 20 ms or the sim stutters)
  const double loopMs =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loopStart).count();
  frc::SmartDashboard::PutNumber("Sim/LoopMs", loopMs);

  // Battery sags under load, like the real one
  frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
      {m_drivetrain.CurrentDraw() + m_intake.CurrentDraw() + m_shooter.CurrentDraw()}));

  PublishTelemetry();
}

void RobotSim::UpdateFieldOptions() {
  const FieldOptions options{frc::SmartDashboard::GetBoolean(kCrossBumpsKey, constants::kRobotCanCrossBumps),
                             frc::SmartDashboard::GetBoolean(kUnderTrenchKey, constants::kRobotFitsUnderTrench)};
  if (options.robotCanCrossBumps != m_fieldOptions.robotCanCrossBumps ||
      options.robotFitsUnderTrench != m_fieldOptions.robotFitsUnderTrench) {
    m_fieldOptions = options;
    m_drivetrain.SetFieldOptions(options);
  }
}

void RobotSim::PublishTelemetry() {
  const frc::Pose2d truePose = m_drivetrain.GetPose();
  const frc::Pose2d estimated = m_hooks.estimatedPose();
  m_truePosePublisher.Set(truePose);
  m_truePose3dPublisher.Set(m_drivetrain.GetPose3d());
  m_poseErrorPublisher.Set(truePose.Translation().Distance(estimated.Translation()).value());
  m_headingErrorPublisher.Set((estimated.Rotation() - truePose.Rotation()).Degrees().value());
  frc::SmartDashboard::PutNumber("Sim/BatteryVolts", frc::RobotController::GetBatteryVoltage().value());

  // Field balls for AdvantageScope (on the carpet). ~400 poses, so only resend
  // when something changed, and at most every kFieldBallPublishLoops loops
  // (sending them every loop while balls roll makes AdvantageScope stutter).
  m_loopsSinceFieldBallPublish++;
  if (!m_fieldBallsPublished ||
      (m_fieldBalls.Version() != m_publishedFieldBallVersion &&
       m_loopsSinceFieldBallPublish >= kFieldBallPublishLoops)) {
    m_loopsSinceFieldBallPublish = 0;
    std::vector<frc::Pose3d> fieldBalls;
    fieldBalls.reserve(m_fieldBalls.Positions().size());
    for (const frc::Translation2d& ball : m_fieldBalls.Positions()) {
      // Sitting on the carpet, or on top of a bump
      const units::meter_t ballZ{constants::kBallDiameterMeters / 2.0 + FloorHeightAt(ball)};
      fieldBalls.emplace_back(ball.X(), ball.Y(), ballZ, frc::Rotation3d{});
    }
    m_fieldBallPublisher.Set(fieldBalls);
    m_publishedFieldBallVersion = m_fieldBalls.Version();
    m_fieldBallsPublished = true;
  }
  // Robot components for AdvantageScope: 0 = turret, 1 = intake arm
  m_componentPublisher.Set(std::vector<frc::Pose3d>{m_shooter.TurretComponent(), m_intake.ArmComponent()});

  frc::SmartDashboard::PutNumber("Sim/Field/BallsOnField", m_fieldBalls.Count());
  frc::SmartDashboard::PutNumber("Sim/Field/Collisions", m_drivetrain.Collisions());
  frc::SmartDashboard::PutNumber("Sim/Field/ContactSeconds", m_drivetrain.ContactSeconds());
  frc::SmartDashboard::PutBoolean("Sim/Field/InContact", m_drivetrain.InContact());
  frc::SmartDashboard::PutNumber("Sim/Intake/ArmDeg", m_intake.ArmDegrees());
  frc::SmartDashboard::PutBoolean("Sim/Intake/Intaking", m_intake.IsIntaking());
  frc::SmartDashboard::PutNumber("Sim/Intake/Collected", m_ballsCollected);
}

void RobotSim::AddCommand(std::string_view name, frc2::CommandPtr command) {
  m_commands.push_back(std::move(command));
  frc::SmartDashboard::PutData("Sim/Commands/" + std::string{name}, m_commands.back().get());
}

void RobotSim::RegisterDashboardCommands() {
  AddCommand("Shoot (aim + spin + fire)", ShootCommand());
  AddCommand("Shoot While Strafing", frc2::cmd::Parallel(ShootCommand(), ShuttleCommand(false)));
  AddCommand("Shoot While Driving At Hub", frc2::cmd::Parallel(ShootCommand(), ShuttleCommand(true)));

  AddCommand("Reset Robot To Start",
             frc2::cmd::RunOnce([this] { m_drivetrain.ResetPose(constants::kStartPose); }).IgnoringDisable(true));
  AddCommand("Snap Odometry To True Pose",
             frc2::cmd::RunOnce([this] { m_hooks.resetOdometry(m_drivetrain.GetPose()); })
                 .IgnoringDisable(true));
  AddCommand("Reset Shot Stats", frc2::cmd::RunOnce([this] { m_shooter.ResetStats(); }).IgnoringDisable(true));
  AddCommand("Refill Balls", frc2::cmd::RunOnce([this] { m_shooter.Refill(); }).IgnoringDisable(true));
  AddCommand("Intake (lower + rollers)", IntakeCommand());
  AddCommand("Move Robot To Shot Test Spot", PlaceRobotCommand([this] {
               return ShotTestPose(frc::SmartDashboard::GetNumber(kShotTestDistanceKey, kDefaultShotTestDistance));
             }).IgnoringDisable(true));
  AddCommand("Shot Sweep (shoot from 1.5-4 m)", ShotSweepCommand());
  AddCommand("Place Robot At Auto Start",
             frc2::cmd::RunOnce([this] { PlaceAtAutoStart(m_hooks.selectedAuto()); }).IgnoringDisable(true));
  AddCommand("Reset Field Balls",
             frc2::cmd::RunOnce([this] { m_fieldBalls.Reset(StartingFieldBalls()); }).IgnoringDisable(true));
}

// The robot's own intake / shooting commands (from the robot binding)
frc2::CommandPtr RobotSim::IntakeCommand() { return m_hooks.intakeCommand(); }
frc2::CommandPtr RobotSim::ShootCommand() { return m_hooks.shootCommand(); }

// Drives back and forth (using the robot's ESTIMATED pose, like real code would):
//   towardHub = false: side to side (y 2.5 - 5.5) at x = Sim/Test/StrafeAtX
//   towardHub = true:  toward / away from the blue hub (x 1.5 - 3.4) at the hub's y
frc2::CommandPtr RobotSim::ShuttleCommand(bool towardHub) {
  auto direction = std::make_shared<double>(1.0);
  return frc2::cmd::Run(
             [this, towardHub, direction] {
               const frc::Pose2d pose = m_hooks.estimatedPose();
               const double speed = frc::SmartDashboard::GetNumber(kShuttleSpeedKey, kDefaultShuttleSpeed);
               const double along = towardHub ? pose.X().value() : pose.Y().value();
               const double low = towardHub ? kTowardHubMinX : kStrafeMinY;
               const double high = towardHub ? kTowardHubMaxX : kStrafeMaxY;
               if (along > high) {
                 *direction = -1.0;
               } else if (along < low) {
                 *direction = 1.0;
               }

               const double lineError =
                   towardHub ? constants::kBlueHubCenter.Y().value() - pose.Y().value()
                             : frc::SmartDashboard::GetNumber(kStrafeXKey, kDefaultStrafeX) - pose.X().value();
               const double hold = std::clamp(kHoldGain * lineError, -kMaxHoldSpeed, kMaxHoldSpeed);
               const double shuttle = *direction * speed;
               const double headingError = (frc::Rotation2d{} - pose.Rotation()).Radians().value();

               const frc::ChassisSpeeds robotRelative = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                   units::meters_per_second_t{towardHub ? shuttle : hold},
                   units::meters_per_second_t{towardHub ? hold : shuttle},
                   units::radians_per_second_t{kHeadingGain * headingError}, pose.Rotation());
               m_hooks.driveRobotRelative(robotRelative);
             },
             {m_hooks.driveSubsystem})
      .FinallyDo([this](bool) { m_hooks.stopDriving(); });
}

// ---------------- Shot testing ----------------

// Robot center `distance` meters from YOUR hub, in the dashboard direction,
// facing the hub (on red, the direction is mirrored like the field).
frc::Pose2d RobotSim::ShotTestPose(double distance) const {
  const bool isRed = m_hooks.isRed();
  const frc::Translation2d hub = isRed ? constants::kRedHubCenter : constants::kBlueHubCenter;
  const double angleDeg = frc::SmartDashboard::GetNumber(kShotTestAngleKey, kDefaultShotTestAngle) +
                          (isRed ? 180.0 : 0.0);
  const frc::Rotation2d direction{units::degree_t{angleDeg}};
  const frc::Translation2d robot = hub + frc::Translation2d{units::meter_t{distance}, direction};
  return frc::Pose2d{robot, direction + frc::Rotation2d{180_deg}};
}

// Picks the robot up, puts it here, and tells the robot code exactly where it is.
// The simulated Pigeon reports the new heading a few loops later (CTRE runs its
// own sim clock), so odometry is reset only after that; resetting it right away
// would leave the robot code with the OLD gyro angle.
frc2::CommandPtr RobotSim::PlaceRobotCommand(std::function<frc::Pose2d()> pose) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, pose] { m_drivetrain.ResetPose(pose()); }),
      frc2::cmd::Wait(kGyroSettleTime),
      frc2::cmd::RunOnce([this] { m_hooks.resetOdometry(m_drivetrain.GetPose()); }));
}

// Runs the robot's real ShootCommand from each distance in kSweepDistances
// (kSweepShotsPerSpot balls each) and reports the hit rate per spot on
// Sim/ShotSweep/... and in the console.
frc2::CommandPtr RobotSim::ShotSweepCommand() {
  std::vector<frc2::CommandPtr> steps;
  steps.push_back(frc2::cmd::RunOnce([this] { m_sweepResults.clear(); }));
  for (double distance : kSweepDistances) {
    steps.push_back(PlaceRobotCommand([this, distance] { return ShotTestPose(distance); }));
    steps.push_back(frc2::cmd::RunOnce([this] {
      m_shooter.SetBallsInRobot(kSweepShotsPerSpot);
      m_sweepBaseline = m_shooter.Stats();
    }));
    steps.push_back(ShootCommand()
                        .Until([this] { return m_shooter.BallsInRobot() == 0 && !m_shooter.BallsInFlight(); })
                        .WithTimeout(kSweepSpotTimeout));
    steps.push_back(frc2::cmd::RunOnce([this, distance] { RecordSweepSpot(distance); }));
  }
  steps.push_back(frc2::cmd::RunOnce([this] { PublishSweepResults(); }));
  return frc2::cmd::Sequence(std::move(steps));
}

void RobotSim::RecordSweepSpot(double distance) {
  const ShotStats& now = m_shooter.Stats();
  SweepResult result;
  result.distance = distance;
  result.fired = now.fired - m_sweepBaseline.fired;
  result.scored = now.scored - m_sweepBaseline.scored;
  const int finished = std::max(now.finished - m_sweepBaseline.finished, 1);
  result.avgLongShort = (now.sumAbsLongShort - m_sweepBaseline.sumAbsLongShort) / finished;
  m_sweepResults.push_back(result);
  PublishSweepResults();
}

void RobotSim::PublishSweepResults() {
  std::vector<double> distances;
  std::vector<double> hitPercent;
  std::vector<double> missMeters;
  std::ostringstream summary;
  for (const SweepResult& r : m_sweepResults) {
    distances.push_back(r.distance);
    hitPercent.push_back(r.fired > 0 ? 100.0 * r.scored / r.fired : 0.0);
    missMeters.push_back(r.avgLongShort);
    summary << r.distance << "m: " << r.scored << "/" << r.fired << "  ";
  }
  frc::SmartDashboard::PutNumberArray("Sim/ShotSweep/DistanceM", distances);
  frc::SmartDashboard::PutNumberArray("Sim/ShotSweep/HitPercent", hitPercent);
  frc::SmartDashboard::PutNumberArray("Sim/ShotSweep/AvgMissLongShortM", missMeters);
  frc::SmartDashboard::PutString("Sim/ShotSweep/Summary", summary.str());
  std::cout << "[Shot sweep] " << summary.str() << std::endl;
}

// ---------------- Autos ----------------

// Like putting the robot on the field before a match: while disabled, keep the
// robot at the start of whichever auto is picked on the dashboard chooser.
void RobotSim::UpdateAutoPlacement() {
  if (m_odometryResetCountdown > 0 && --m_odometryResetCountdown == 0) {
    m_hooks.resetOdometry(m_drivetrain.GetPose());
  }
  if (!frc::DriverStation::IsDisabled() || !frc::SmartDashboard::GetBoolean(kPlaceAtAutoStartKey, true)) {
    return;
  }
  const std::string selected = m_hooks.selectedAuto();
  if (selected != m_placedAuto) {
    PlaceAtAutoStart(selected);
  }
}

void RobotSim::PlaceAtAutoStart(const std::string& name) {
  m_placedAuto = name;
  if (name.empty()) {
    return;
  }
  const frc::Pose2d start = m_hooks.autoStartPose(name);
  m_autoStartPublisher.Set(start);
  m_drivetrain.ResetPose(start);
  m_odometryResetCountdown = kGyroSettleLoops;  // then tell the robot code where it is
  m_shooter.SetBallsInRobot(constants::kStartingBalls);
}

// When autonomous ends: what the auto did (Sim/Auto/...).
void RobotSim::UpdateAutoReport() {
  const bool autoEnabled = frc::DriverStation::IsAutonomousEnabled();
  if (autoEnabled && !m_wasAutonomousEnabled) {
    m_autoStartStats = m_shooter.Stats();
    m_autoStartCollected = m_ballsCollected;
    m_autoStartSeconds = frc::Timer::GetFPGATimestamp().value();
    m_drivetrain.ResetCollisions();
  }
  if (!autoEnabled && m_wasAutonomousEnabled) {
    const ShotStats& now = m_shooter.Stats();
    std::ostringstream summary;
    summary << m_placedAuto << ": scored " << now.scored - m_autoStartStats.scored << "/"
            << now.fired - m_autoStartStats.fired << ", picked up " << m_ballsCollected - m_autoStartCollected
            << ", hit walls " << m_drivetrain.Collisions() << "x (" << m_drivetrain.ContactSeconds()
            << " s), ran " << frc::Timer::GetFPGATimestamp().value() - m_autoStartSeconds << " s";
    frc::SmartDashboard::PutString("Sim/Auto/LastRun", summary.str());
    std::cout << "[Auto] " << summary.str() << std::endl;
  }
  m_wasAutonomousEnabled = autoEnabled;
}

}  // namespace robotsim
