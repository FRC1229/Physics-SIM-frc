// ================================================================
// Shooting "bots": play the copilot with the REAL robot code and shoot from
// many spots (standing still and on the move). Every shot is recorded.
//
// These are slow (real time), so they are DISABLED_ and only run on purpose:
//   frcUserProgramTest.exe --gtest_also_run_disabled_tests --gtest_filter=ShotTuning*
//
// Environment variables:
//   SHOT_TUNE      "Shooter/LatencySec=0.12;Shooter/RpmOffset=-20"  dashboard values to try
//   SHOT_TAG       name for the CSV file (default "run")
//   SHOT_ALLIANCE  "red" to shoot at the red hub (default blue)
// Output: build/shot_tuning/<tag>_<stationary|moving>.csv + a summary in the console.
// ================================================================

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStationTypes.h>

#include "SimTestHarness.h"
#include "sim/FieldGeometry.h"

using namespace robotsim;

namespace {

constexpr int kShootButton = 5;   // copilot LB: ShooterCommand + AutoLock
constexpr int kLaunchButton = 6;  // copilot RB: LaunchCommand (feed)
constexpr double kReadyTurretDeg = 2.0;
constexpr double kMaxSpinUpSeconds = 3.0;
constexpr double kMaxFireSeconds = 4.0;
constexpr double kZoneMaxX = 4.3;  // stay inside the code's shooting zone (x < 4.5) with margin

std::string Env(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && *value != '\0' ? std::string{value} : fallback;
}

bool IsRed() { return Env("SHOT_ALLIANCE", "blue") == "red"; }

// SHOT_TUNE: "key=value;key=value" -> SmartDashboard (true/false become booleans)
void ApplyTuning() {
  std::stringstream entries{Env("SHOT_TUNE", "")};
  std::string entry;
  while (std::getline(entries, entry, ';')) {
    const size_t eq = entry.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = entry.substr(0, eq);
    const std::string value = entry.substr(eq + 1);
    if (value == "true" || value == "false") {
      frc::SmartDashboard::PutBoolean(key, value == "true");
    } else {
      frc::SmartDashboard::PutNumber(key, std::stod(value));
    }
    std::cout << "[tune] " << key << " = " << value << "\n";
  }
}

// Blue-side point / heading turned onto the red side (the field is rotated 180 deg)
frc::Translation2d ForAlliance(const frc::Translation2d& blue) {
  if (!IsRed()) {
    return blue;
  }
  return frc::Translation2d{units::meter_t{constants::kFieldLengthMeters} - blue.X(),
                            units::meter_t{constants::kFieldWidthMeters} - blue.Y()};
}
frc::Rotation2d ForAlliance(const frc::Rotation2d& blue) {
  return IsRed() ? blue + frc::Rotation2d{180_deg} : blue;
}

struct LabeledShot {
  std::string scenario;
  ShotRecord record;
};

// Robot pose this far from the hub center, in this direction (blue side,
// 180 = toward the alliance wall), turned headingOffsetDeg away from facing the hub.
std::optional<frc::Pose2d> SpotNearHub(double distance, double angleDeg, double headingOffsetDeg) {
  const frc::Rotation2d direction{units::degree_t{angleDeg}};
  const frc::Translation2d blue =
      constants::kBlueHubCenter + frc::Translation2d{units::meter_t{distance}, direction};
  const frc::Rotation2d facingHub = direction + frc::Rotation2d{180_deg};
  const frc::Pose2d bluePose{blue, facingHub + frc::Rotation2d{units::degree_t{headingOffsetDeg}}};
  if (bluePose.X().value() > kZoneMaxX) {
    return std::nullopt;
  }
  // Skip spots where the robot would be inside a wall or structure
  const frc::Pose2d resolved = ResolveRobotCollisions(bluePose, constants::kBumperLengthMeters / 2.0,
                                                      constants::kBumperWidthMeters / 2.0,
                                                      RobotObstacles(FieldOptions{constants::kRobotCanCrossBumps,
                                                                                  constants::kRobotFitsUnderTrench}));
  if (resolved.Translation().Distance(bluePose.Translation()).value() > 0.01) {
    return std::nullopt;
  }
  return frc::Pose2d{ForAlliance(blue), ForAlliance(bluePose.Rotation())};
}

}  // namespace

class ShotTuningTest : public simtest::SimFixture {
 protected:
  void SetUp() override {
    SimFixture::SetUp();
    frc::sim::DriverStationSim::SetAllianceStationId(IsRed() ? HAL_AllianceStationID_kRed1
                                                             : HAL_AllianceStationID_kBlue1);
    frc::sim::DriverStationSim::NotifyNewData();
    s_robot->m_vision.isRed = IsRed();
    ApplyTuning();
    s_sim->GetFieldBalls().Clear();  // nothing on the carpet to bump into
  }

  // What a good copilot waits for before pulling the trigger
  static bool ReadyToFire() {
    TurretSubsystem& turret = s_robot->m_turret;
    const double turretError = turret.shortestAngleError(turret.targetAngle, turret.readEncoder());
    return s_robot->m_shooter.AtSetpoint() && std::abs(turretError) < kReadyTurretDeg;
  }

  // Runs the robot until the condition is true or the time runs out.
  template <typename Condition>
  static bool WaitFor(Condition condition, double maxSeconds) {
    for (double t = 0.0; t < maxSeconds; t += 0.02) {
      if (condition()) {
        return true;
      }
      RunOneLoop();
    }
    return condition();
  }

  static bool AllShotsDone() {
    return s_sim->Shooter().BallsInRobot() == 0 && !s_sim->Shooter().BallsInFlight();
  }

  void Collect(const std::string& scenario, size_t firstRecord) {
    const std::vector<ShotRecord>& records = s_sim->Shooter().Records();
    for (size_t i = firstRecord; i < records.size(); ++i) {
      m_shots.push_back(LabeledShot{scenario, records[i]});
    }
  }

  void WriteCsv(const std::string& kind) const {
    const std::filesystem::path dir = std::filesystem::path{"build"} / "shot_tuning";
    std::filesystem::create_directories(dir);
    const std::filesystem::path file = dir / (Env("SHOT_TAG", "run") + "_" + kind + ".csv");
    std::ofstream out{file};
    out << "scenario,x,y,heading_deg,vx,vy,omega,speed,flywheel_rpm,target_rpm,turret_deg,distance,"
           "finished,outcome,long_short_m,left_right_m,flight_s\n";
    for (const LabeledShot& shot : m_shots) {
      const ShotRecord& r = shot.record;
      out << shot.scenario << "," << r.robotPose.X().value() << "," << r.robotPose.Y().value() << ","
          << r.robotPose.Rotation().Degrees().value() << "," << r.fieldSpeeds.vx.value() << ","
          << r.fieldSpeeds.vy.value() << "," << r.fieldSpeeds.omega.value() << ","
          << std::hypot(r.fieldSpeeds.vx.value(), r.fieldSpeeds.vy.value()) << "," << r.flywheelRpm << ","
          << r.targetRpm << "," << r.turretRobotDeg << "," << r.distanceToHub << "," << r.finished << ","
          << ToString(r.result.outcome) << "," << r.result.longShortMeters << ","
          << r.result.leftRightMeters << "," << r.result.flightSeconds << "\n";
    }
    std::cout << "[shot tuning] wrote " << m_shots.size() << " shots to " << file.string() << "\n";
  }

  // Hit rate per bucket, plus the average miss direction (shows what to fix).
  template <typename BucketOf>
  void PrintSummary(const std::string& title, BucketOf bucketOf) const {
    struct Totals {
      int shots = 0;
      int scored = 0;
      double longShort = 0.0;
      double leftRight = 0.0;
      double rpmError = 0.0;
    };
    std::map<double, Totals> buckets;
    Totals all;
    for (const LabeledShot& shot : m_shots) {
      const ShotRecord& r = shot.record;
      if (!r.finished) {
        continue;
      }
      for (Totals* t : {&buckets[bucketOf(r)], &all}) {
        t->shots++;
        t->scored += r.result.outcome == ShotOutcome::kScored ? 1 : 0;
        t->longShort += r.result.longShortMeters;
        t->leftRight += r.result.leftRightMeters;
        t->rpmError += r.flywheelRpm - r.targetRpm;
      }
    }
    std::cout << std::fixed << std::setprecision(2) << "\n=== " << title << " ===\n"
              << " bucket   hit%  shots  avgLong(+)/Short(-)m  avgLeft(+)/Right(-)m  avgRpmErr\n";
    auto line = [](const std::string& name, const Totals& t) {
      const double n = std::max(t.shots, 1);
      std::cout << std::setw(7) << name << " " << std::setw(6) << 100.0 * t.scored / n << " " << std::setw(6)
                << t.shots << " " << std::setw(20) << t.longShort / n << " " << std::setw(21)
                << t.leftRight / n << " " << std::setw(10) << t.rpmError / n << "\n";
    };
    for (const auto& [bucket, totals] : buckets) {
      std::ostringstream name;
      name << std::fixed << std::setprecision(1) << bucket;
      line(name.str(), totals);
    }
    line("ALL", all);
  }

  std::vector<LabeledShot> m_shots;
};

// Standing still: 9 distances x 5 directions, robot turned so the turret works its range.
TEST_F(ShotTuningTest, DISABLED_StationaryGrid) {
  constexpr int kBallsPerSpot = 4;
  const std::vector<double> distances{1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5};
  const std::vector<double> angles{120.0, 150.0, 180.0, 210.0, 240.0};
  const std::vector<double> headingOffsets{-60.0, 0.0, 60.0};
  size_t spot = 0;
  for (double distance : distances) {
    for (double angle : angles) {
      const double headingOffset = headingOffsets[spot++ % headingOffsets.size()];
      const std::optional<frc::Pose2d> pose = SpotNearHub(distance, angle, headingOffset);
      if (!pose.has_value()) {
        continue;
      }
      PlaceRobot(pose.value());
      s_sim->Shooter().SetBallsInRobot(kBallsPerSpot);
      RunFor(0.2);
      const size_t first = s_sim->Shooter().Records().size();

      SetButton(kCopilotPort, kShootButton, true);
      WaitFor(ReadyToFire, kMaxSpinUpSeconds);
      SetButton(kCopilotPort, kLaunchButton, true);
      WaitFor(AllShotsDone, kMaxFireSeconds);
      ReleaseAllButtons();
      WaitFor([] { return !s_sim->Shooter().BallsInFlight(); }, 3.0);

      std::ostringstream name;
      name << "d" << distance << "_a" << angle << "_h" << headingOffset;
      Collect(name.str(), first);
    }
  }
  WriteCsv("stationary");
  PrintSummary("STANDING STILL, by distance (m)",
               [](const ShotRecord& r) { return std::round(r.distanceToHub * 2.0) / 2.0; });
}

// On the move: spin up standing still, then drive a straight line while feeding.
TEST_F(ShotTuningTest, DISABLED_MovingShots) {
  struct Run {
    std::string name;
    frc::Translation2d start;  // blue side
    double directionDeg;       // blue side, field direction of travel
    double omega;              // rad/s
    double travelMeters;       // how far to drive
  };
  // Every path stays inside the shooting zone (x < 4.3), clear of the hub and
  // tower, and at least ~2.2 m from the hub center (closer, the code refuses:
  // a 60 deg hood can't clear the rim).
  const std::vector<Run> runs{
      {"strafe_left", frc::Translation2d{1.9_m, 1.6_m}, 90.0, 0.0, 3.0},
      {"strafe_right", frc::Translation2d{1.9_m, 6.4_m}, -90.0, 0.0, 3.0},
      {"toward_hub", frc::Translation2d{0.9_m, 6.0_m}, -28.0, 0.0, 2.0},
      {"away_from_hub", frc::Translation2d{2.4_m, 5.0_m}, 156.0, 0.0, 2.0},
      {"diagonal", frc::Translation2d{1.0_m, 1.2_m}, 25.0, 0.0, 2.2},
      {"strafe_spinning", frc::Translation2d{1.9_m, 1.6_m}, 90.0, 1.5, 3.0},
  };

  const std::vector<double> speeds{1.0, 2.0, 3.0};
  constexpr int kBallsPerRun = 6;

  for (double speed : speeds) {
    for (const Run& run : runs) {
      const frc::Translation2d start = ForAlliance(run.start);
      const frc::Rotation2d facingHub = (ForAlliance(constants::kBlueHubCenter) - start).Angle();
      PlaceRobot(frc::Pose2d{start, facingHub});
      s_sim->Shooter().SetBallsInRobot(kBallsPerRun);
      RunFor(0.2);
      const size_t first = s_sim->Shooter().Records().size();

      SetButton(kCopilotPort, kShootButton, true);
      WaitFor(ReadyToFire, kMaxSpinUpSeconds);

      // Field-relative velocity, turned into robot-relative every loop
      const frc::Rotation2d travel = ForAlliance(frc::Rotation2d{units::degree_t{run.directionDeg}});
      const double vx = speed * travel.Cos();
      const double vy = speed * travel.Sin();
      const double omega = run.omega;
      frc2::CommandPtr drive = frc2::cmd::Run(
          [vx, vy, omega] {
            const frc::ChassisSpeeds robot = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                units::radians_per_second_t{omega}, s_robot->m_drive.GetEstimatedPose().Rotation());
            s_robot->m_drive.Drive(robot.vx, robot.vy, robot.omega, false);
          },
          {&s_robot->m_drive});
      frc2::CommandScheduler::GetInstance().Schedule(drive);
      RunFor(0.4);  // get up to speed before the first ball
      SetButton(kCopilotPort, kLaunchButton, true);
      const double driveSeconds = std::min(kMaxFireSeconds, run.travelMeters / speed);
      // While the trigger is held: how often each part of "ready to fire" was true
      int loops = 0;
      int atSpeed = 0;
      int valid = 0;
      int onTarget = 0;
      double worstTurretError = 0.0;
      WaitFor(
          [&] {
            TurretSubsystem& turret = s_robot->m_turret;
            const double error = std::abs(turret.shortestAngleError(turret.targetAngle, turret.readEncoder()));
            loops++;
            if (Env("SHOT_TRACE", "") == run.name + "_v" + std::to_string(static_cast<int>(speed))) {
              const frc::ChassisSpeeds v = s_sim->GetFieldRelativeSpeeds();
              std::cout << "[trace] t " << loops * 0.02 << " target " << turret.targetAngle << " actual "
                        << turret.readEncoder() << " ff " << frc::SmartDashboard::GetNumber("SOTM TurretFF", 0)
                        << " sotm " << frc::SmartDashboard::GetNumber("SOTM TurretDeg", 0) << " valid "
                        << s_robot->m_shooter.HasValidShot() << " speed " << std::hypot(v.vx.value(), v.vy.value())
                        << " omega " << v.omega.value() << "\n";
            }
            atSpeed += s_robot->m_shooter.AtSetpoint() ? 1 : 0;
            valid += s_robot->m_shooter.HasValidShot() ? 1 : 0;
            onTarget += error <= frc::SmartDashboard::GetNumber("Shooter/FeedTurretToleranceDeg", 3.0) ? 1 : 0;
            worstTurretError = std::max(worstTurretError, error);
            return s_sim->Shooter().BallsInRobot() == 0;
          },
          driveSeconds);
      std::cout << std::fixed << std::setprecision(0) << "[moving] " << run.name << " v" << speed
                << ": at speed " << 100.0 * atSpeed / std::max(loops, 1) << "%, valid "
                << 100.0 * valid / std::max(loops, 1) << "%, turret on target " << 100.0 * onTarget / std::max(loops, 1)
                << "% (worst " << worstTurretError << " deg), fired "
                << kBallsPerRun - s_sim->Shooter().BallsInRobot() << "/" << kBallsPerRun << "\n";
      ReleaseAllButtons();
      drive.Cancel();
      DriveFor(0.0, 0.0, 0.0, 0.1);
      WaitFor([] { return !s_sim->Shooter().BallsInFlight(); }, 3.0);

      std::ostringstream name;
      name << run.name << "_v" << speed;
      Collect(name.str(), first);
    }
  }
  WriteCsv("moving");
  PrintSummary("ON THE MOVE, by robot speed (m/s)", [](const ShotRecord& r) {
    return std::round(std::hypot(r.fieldSpeeds.vx.value(), r.fieldSpeeds.vy.value()) * 2.0) / 2.0;
  });
  PrintSummary("ON THE MOVE, by distance (m)",
               [](const ShotRecord& r) { return std::round(r.distanceToHub * 2.0) / 2.0; });
}
