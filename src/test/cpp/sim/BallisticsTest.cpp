#include <cmath>
#include <numbers>
#include <optional>

#include <gtest/gtest.h>

#include "Constants.h"
#include "sim/Ballistics.h"
#include "sim/SimConstants.h"

using namespace robotsim;

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

Hub BlueHub() {
  return Hub{constants::kBlueHubCenter, constants::kHubOpeningHeightMeters,
             constants::kHubOpeningRadiusMeters, constants::kHubOuterRadiusMeters};
}

BallPhysics RealPhysics() {
  return BallPhysics{constants::kGravity, constants::kDragFactor, constants::kBallDiameterMeters / 2.0};
}

// Fires a ball from `distance` meters in front of the blue hub (on the hub's
// line, aimed straight at it) and flies it until it finishes.
ShotResult ShootFrom(double distance, double rpm, double robotVx = 0.0) {
  const Hub hub = BlueHub();
  Ball ball;
  ball.position = Vec3{hub.center.X().value() - distance, hub.center.Y().value(),
                       constants::kLaunchHeightMeters};
  ball.launchPosition = ball.position;
  ball.velocity = LaunchVelocity(rpm * constants::kExitSpeedPerRpm, constants::kHoodAngleDeg * kDegToRad,
                                 0.0, robotVx, 0.0);
  for (int i = 0; i < 500; ++i) {
    std::optional<ShotResult> result = StepBall(ball, 0.02, hub, RealPhysics());
    if (result.has_value()) {
      return result.value();
    }
  }
  ADD_FAILURE() << "ball never finished";
  return ShotResult{};
}

double Interpolate(const std::vector<double>& keys, const std::vector<double>& values, double key) {
  for (size_t i = 1; i < keys.size(); ++i) {
    if (key <= keys[i]) {
      const double t = (key - keys[i - 1]) / (keys[i] - keys[i - 1]);
      return values[i - 1] + t * (values[i] - values[i - 1]);
    }
  }
  return values.back();
}

}  // namespace

TEST(BallisticsTest, LaunchVelocityAddsRobotMotion) {
  const Vec3 v = LaunchVelocity(10.0, 45.0 * kDegToRad, 0.0, 2.0, -1.0);
  EXPECT_NEAR(v.x, 10.0 * std::cos(45.0 * kDegToRad) + 2.0, 1e-9);
  EXPECT_NEAR(v.y, -1.0, 1e-9);
  EXPECT_NEAR(v.z, 10.0 * std::sin(45.0 * kDegToRad), 1e-9);
}

TEST(BallisticsTest, NoDragMatchesProjectileFormula) {
  // Straight up and down from the opening height: lands back after 2*v/g
  Hub farHub{frc::Translation2d{100_m, 100_m}, 1.0, 0.5, 0.6};
  Ball ball;
  ball.position = Vec3{0.0, 0.0, 1.0 + 1e-9};
  ball.launchPosition = ball.position;
  ball.velocity = Vec3{3.0, 0.0, 5.0};
  BallPhysics noDrag{9.81, 0.0, 0.075};

  std::optional<ShotResult> result;
  while (!result.has_value()) {
    result = StepBall(ball, 0.02, farHub, noDrag);
  }
  const double expectedTime = 2.0 * 5.0 / 9.81;
  EXPECT_NEAR(result->flightSeconds, expectedTime, 0.01);
  EXPECT_NEAR(result->endPosition.X().value(), 3.0 * expectedTime, 0.03);
}

TEST(BallisticsTest, BallDroppedIntoHubCenterScores) {
  const Hub hub = BlueHub();
  Ball ball;
  ball.position = Vec3{hub.center.X().value(), hub.center.Y().value(), 3.0};
  ball.launchPosition = Vec3{hub.center.X().value() - 3.0, hub.center.Y().value(), 0.5};
  std::optional<ShotResult> result;
  while (!result.has_value()) {
    result = StepBall(ball, 0.02, hub, RealPhysics());
  }
  EXPECT_EQ(result->outcome, ShotOutcome::kScored);
}

TEST(BallisticsTest, WeakShotFallsShortAndStrongShotGoesLong) {
  // 1200 RPM never even gets up to the opening height: lands on the carpet short of the hub
  const ShotResult weak = ShootFrom(4.0, 1200);
  EXPECT_EQ(weak.outcome, ShotOutcome::kHitFloor);
  EXPECT_LT(weak.longShortMeters, 0.0);
  // 1700 RPM gets high enough but not far enough: comes down short of the opening
  const ShotResult slightlyWeak = ShootFrom(4.0, 1700);
  EXPECT_NE(slightlyWeak.outcome, ShotOutcome::kScored);
  EXPECT_LT(slightlyWeak.longShortMeters, 0.0);
  EXPECT_EQ(ShootFrom(4.0, 3000).outcome, ShotOutcome::kMissedLong);
}

// The sim's exit speed was fitted to the team's RPM table (60 deg measured hood).
// Stationary shots at the table's RPM must score from 2.25 to 6 m. (Closer than
// ~2 m a 60 deg shot can't clear the hub's rim at any RPM.)
TEST(BallisticsTest, TeamRpmTableScoresWhenStandingStill) {
  for (double distance = 2.25; distance <= 6.0; distance += 0.25) {
    const double rpm = Interpolate(ShooterConstants::sotm_distance_dim, ShooterConstants::sotm_rpms, distance);
    const ShotResult result = ShootFrom(distance, rpm);
    EXPECT_EQ(result.outcome, ShotOutcome::kScored)
        << "distance " << distance << " m, rpm " << rpm << ": " << ToString(result.outcome)
        << " by " << result.longShortMeters << " m";
  }
}

TEST(BallisticsTest, DrivingTowardHubWithoutLeadOvershoots) {
  const ShotResult still = ShootFrom(4.0, Interpolate(ShooterConstants::sotm_distance_dim,
                                                      ShooterConstants::sotm_rpms, 4.0));
  const ShotResult moving = ShootFrom(4.0, Interpolate(ShooterConstants::sotm_distance_dim,
                                                       ShooterConstants::sotm_rpms, 4.0),
                                      2.0);
  EXPECT_GT(moving.longShortMeters, still.longShortMeters + 1.0);
}

TEST(BallisticsTest, MissDirectionIsReportedRelativeToShot) {
  const Hub hub = BlueHub();
  Ball ball;
  // Comes down 1 m to the LEFT of the hub (as seen from a shooter at -x)
  ball.position = Vec3{hub.center.X().value(), hub.center.Y().value() + 1.0, 3.0};
  ball.launchPosition = Vec3{hub.center.X().value() - 3.0, hub.center.Y().value(), 0.5};
  std::optional<ShotResult> result;
  while (!result.has_value()) {
    result = StepBall(ball, 0.02, hub, RealPhysics());
  }
  EXPECT_NEAR(result->leftRightMeters, 1.0, 0.01);
  EXPECT_NEAR(result->longShortMeters, 0.0, 0.01);
}

namespace {

struct FinishedShot {
  ShotResult result;
  Ball ball;
};

FinishedShot FlyUntilFinished(Ball ball, const Hub& hub) {
  for (int i = 0; i < 500; ++i) {
    std::optional<ShotResult> result = StepBall(ball, 0.02, hub, RealPhysics());
    if (result.has_value()) {
      return FinishedShot{result.value(), ball};
    }
  }
  ADD_FAILURE() << "ball never finished";
  return FinishedShot{};
}

Ball LaunchedFrom(double distance, double rpm) {
  const Hub hub = BlueHub();
  Ball ball;
  ball.position = Vec3{hub.center.X().value() - distance, hub.center.Y().value(),
                       constants::kLaunchHeightMeters};
  ball.launchPosition = ball.position;
  ball.velocity = LaunchVelocity(rpm * constants::kExitSpeedPerRpm, constants::kHoodAngleDeg * kDegToRad,
                                 0.0, 0.0, 0.0);
  return ball;
}

}  // namespace

TEST(BallisticsTest, ScoredBallComesOutOfTheHubExit) {
  Hub hub = BlueHub();
  hub.exitPoint = frc::Translation2d{hub.center.X() + 1.2_m, hub.center.Y()};
  Ball ball;
  ball.position = Vec3{hub.center.X().value(), hub.center.Y().value(), 3.0};
  ball.launchPosition = Vec3{hub.center.X().value() - 3.0, hub.center.Y().value(), 0.5};
  const FinishedShot shot = FlyUntilFinished(ball, hub);

  ASSERT_EQ(shot.result.outcome, ShotOutcome::kScored);
  const frc::Translation2d rest = LandBall(shot.result, shot.ball, hub, RealPhysics()).position;
  EXPECT_NEAR(rest.X().value(), hub.exitPoint.X().value(), 1e-9);
  EXPECT_NEAR(rest.Y().value(), hub.exitPoint.Y().value(), 1e-9);
}

TEST(BallisticsTest, LongMissLandsBeyondTheHub) {
  const Hub hub = BlueHub();
  const FinishedShot shot = FlyUntilFinished(LaunchedFrom(4.0, 3000), hub);

  ASSERT_EQ(shot.result.outcome, ShotOutcome::kMissedLong);
  const frc::Translation2d rest = LandBall(shot.result, shot.ball, hub, RealPhysics()).position;
  EXPECT_GT(rest.X().value(), hub.center.X().value() + hub.outerRadius)
      << "a long miss should land on the far side of the hub";
}

TEST(BallisticsTest, BallNeverRestsInsideTheHubBody) {
  const Hub hub = BlueHub();
  for (double rpm = 1200; rpm <= 3200; rpm += 100) {
    const FinishedShot shot = FlyUntilFinished(LaunchedFrom(4.0, rpm), hub);
    const frc::Translation2d rest = LandBall(shot.result, shot.ball, hub, RealPhysics()).position;
    if (shot.result.outcome == ShotOutcome::kScored) {
      continue;  // goes to the exit point (default-constructed here)
    }
    EXPECT_GE(rest.Distance(hub.center).value(), hub.outerRadius + constants::kBallDiameterMeters / 2.0 - 1e-6)
        << "rpm " << rpm << ": " << ToString(shot.result.outcome);
  }
}
