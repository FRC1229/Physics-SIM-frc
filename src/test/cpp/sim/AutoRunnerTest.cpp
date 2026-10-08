// ================================================================
// Auto bot: runs the robot's PathPlanner autos with the REAL robot code, like
// a match: robot placed at the auto's start, autonomous enabled for 20 s.
// Reports what each auto did (and writes build/auto_runs/autos.csv).
//
// Real time (~22 s per auto), so it only runs on purpose:
//   frcUserProgramTest.exe --gtest_also_run_disabled_tests --gtest_filter=AutoRunner*
// AUTO_NAMES="HomeDepot,DepotCleanup" runs only those (default: every .auto file).
// SIM_SETTINGS="Sim/Field/RobotCanCrossBumps=true" sets dashboard values first.
// ================================================================

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStationTypes.h>

#include "SimTestHarness.h"

using namespace robotsim;

namespace {

constexpr double kAutoSeconds = 20.0;   // 2026 autonomous period
constexpr double kSettleSeconds = 1.5;  // let the last shots land

std::vector<std::string> AutoNames() {
  std::vector<std::string> names;
  const char* wanted = std::getenv("AUTO_NAMES");
  if (wanted != nullptr && *wanted != '\0') {
    std::stringstream list{wanted};
    std::string name;
    while (std::getline(list, name, ',')) {
      names.push_back(name);
    }
    return names;
  }
  for (const auto& entry : std::filesystem::directory_iterator{"src/main/deploy/pathplanner/autos"}) {
    if (entry.path().extension() == ".auto") {
      names.push_back(entry.path().stem().string());
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

struct AutoRun {
  std::string name;
  bool finished = false;
  double seconds = 0.0;
  frc::Pose2d end;
  double poseError = 0.0;
  int fired = 0;
  int scored = 0;
  int pickedUp = 0;
  int collisions = 0;
  double contactSeconds = 0.0;
  std::optional<frc::Pose2d> firstHit;
  std::string misses;  // how the missed shots missed
};

// SIM_SETTINGS: "key=value;key=value" -> SmartDashboard (true/false become booleans)
void ApplySettings() {
  const char* settings = std::getenv("SIM_SETTINGS");
  std::stringstream entries{settings != nullptr ? settings : ""};
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
  }
}

void SetAutonomousEnabled(bool enabled) {
  frc::sim::DriverStationSim::SetAutonomous(enabled);
  frc::sim::DriverStationSim::SetEnabled(enabled);
  frc::sim::DriverStationSim::NotifyNewData();
}

}  // namespace

class AutoRunnerTest : public simtest::SimFixture {
 protected:
  void SetUp() override {
    SimFixture::SetUp();
    frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue1);
    frc::sim::DriverStationSim::NotifyNewData();
    ApplySettings();
    RunFor(0.1);  // field options are applied on the next sim loop
  }

  AutoRun Run(const std::string& name) {
    AutoRun run;
    run.name = name;
    SetEnabled(false);
    s_sim->GetFieldBalls().Reset(StartingFieldBalls());
    s_sim->Shooter().SetBallsInRobot(constants::kStartingBalls);
    // Fresh "match": intake stowed and the robot code's intake target back to raised
    // (its "Intake move" command toggles, so a leftover state would flip it)
    s_sim->ResetMechanisms();
    s_robot->m_intake.SetTarget(IntakeSubsystem::kRaised);
    PlaceRobot(s_sim->AutoStartPose(name));
    s_sim->Drivetrain().ResetCollisions();
    const ShotStats before = s_sim->Shooter().Stats();
    const size_t firstShot = s_sim->Shooter().Records().size();
    const int collectedBefore = s_sim->BallsCollected();

    SetAutonomousEnabled(true);
    frc2::CommandPtr autoCommand = s_sim->AutoCommand(name);
    frc2::CommandScheduler::GetInstance().Schedule(autoCommand);
    for (; run.seconds < kAutoSeconds; run.seconds += 0.02) {
      RunOneLoop();
      if (!autoCommand.IsScheduled()) {
        run.finished = true;
        break;
      }
    }
    autoCommand.Cancel();
    SetAutonomousEnabled(false);
    RunFor(kSettleSeconds);

    const ShotStats& after = s_sim->Shooter().Stats();
    run.end = s_sim->GetTruePose();
    run.poseError = run.end.Translation().Distance(s_robot->m_drive.GetEstimatedPose().Translation()).value();
    run.fired = after.fired - before.fired;
    run.scored = after.scored - before.scored;
    run.pickedUp = s_sim->BallsCollected() - collectedBefore;
    run.collisions = s_sim->Drivetrain().Collisions();
    run.contactSeconds = s_sim->Drivetrain().ContactSeconds();
    run.firstHit = s_sim->Drivetrain().FirstContact();
    std::ostringstream misses;
    const std::vector<ShotRecord>& records = s_sim->Shooter().Records();
    for (size_t i = firstShot; i < records.size(); ++i) {
      const ShotRecord& shot = records[i];
      if (shot.finished && shot.result.outcome != ShotOutcome::kScored) {
        misses << ToString(shot.result.outcome) << " (" << std::fixed << std::setprecision(1) << shot.distanceToHub
               << " m out, long " << shot.result.longShortMeters << ", left " << shot.result.leftRightMeters
               << ", turret " << shot.turretRobotDeg << " deg, rpm " << shot.flywheelRpm << "/" << shot.targetRpm
               << ")  ";
      }
    }
    run.misses = misses.str();
    return run;
  }
};

TEST_F(AutoRunnerTest, DISABLED_RunAutos) {
  std::vector<AutoRun> runs;
  for (const std::string& name : AutoNames()) {
    std::cout << "[auto] running " << name << " ..." << std::endl;
    runs.push_back(Run(name));
  }

  const std::filesystem::path dir = std::filesystem::path{"build"} / "auto_runs";
  std::filesystem::create_directories(dir);
  std::ofstream csv{dir / "autos.csv"};
  csv << "auto,finished,seconds,end_x,end_y,end_heading_deg,pose_error_m,fired,scored,picked_up,collisions,contact_s\n";

  std::cout << std::fixed << std::setprecision(1) << "\n=== AUTOS ===\n"
            << std::left << std::setw(22) << "auto" << std::right
            << "  done   time   end (x, y, deg)          poseErr  scored  pickedUp  wallHits\n";
  for (const AutoRun& r : runs) {
    std::cout << std::left << std::setw(22) << r.name << std::right << "  " << (r.finished ? "yes" : " NO") << "  "
              << std::setw(5) << r.seconds << "s  (" << std::setw(5) << r.end.X().value() << ", " << std::setw(4)
              << r.end.Y().value() << ", " << std::setw(6) << r.end.Rotation().Degrees().value() << ")   "
              << std::setprecision(2) << std::setw(6) << r.poseError << "m  " << std::setw(3) << r.scored << "/"
              << std::setw(2) << r.fired << "  " << std::setw(6) << r.pickedUp << "  " << std::setw(5)
              << r.collisions << " (" << std::setprecision(1) << r.contactSeconds << "s)";
    if (r.firstHit) {
      std::cout << "  first hit at (" << r.firstHit->X().value() << ", " << r.firstHit->Y().value() << ")";
    }
    std::cout << "\n";
    if (!r.misses.empty()) {
      std::cout << "      misses: " << r.misses << "\n";
    }
    csv << r.name << "," << r.finished << "," << r.seconds << "," << r.end.X().value() << "," << r.end.Y().value()
        << "," << r.end.Rotation().Degrees().value() << "," << r.poseError << "," << r.fired << "," << r.scored
        << "," << r.pickedUp << "," << r.collisions << "," << r.contactSeconds << "\n";
  }
  std::cout << "[auto] wrote " << (dir / "autos.csv").string() << std::endl;
}
