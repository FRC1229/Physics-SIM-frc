#include <cmath>

#include <gtest/gtest.h>

#include "sim/FieldBalls.h"
#include "sim/FieldGeometry.h"
#include "sim/SimConstants.h"

using namespace robotsim;

namespace {

constexpr double kHalfLength = constants::kBumperLengthMeters / 2.0;
constexpr double kHalfWidth = constants::kBumperWidthMeters / 2.0;
constexpr double kBallRadius = constants::kBallDiameterMeters / 2.0;

frc::Translation2d At(double x, double y) {
  return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
}

RobotBody ParkedRobot(const frc::Pose2d& pose) {
  return RobotBody{pose, frc::ChassisSpeeds{}, kHalfLength, kHalfWidth};
}

}  // namespace

TEST(FieldGeometryTest, RobotDrivenIntoTheHubStopsAtItsFace) {
  const double hubFrontX = constants::kBlueHubCenter.X().value() - constants::kHubHalfSizeMeters;
  // Robot center 0.2 m inside where its front bumper would touch the hub
  const frc::Pose2d inside{units::meter_t{hubFrontX - kHalfLength + 0.2}, constants::kBlueHubCenter.Y(),
                           frc::Rotation2d{}};
  const frc::Pose2d resolved = ResolveRobotCollisions(inside, kHalfLength, kHalfWidth, RobotObstacles({}));
  EXPECT_NEAR(resolved.X().value(), hubFrontX - kHalfLength, 1e-6);
  EXPECT_NEAR(resolved.Y().value(), constants::kBlueHubCenter.Y().value(), 1e-6);
}

TEST(FieldGeometryTest, TurnedRobotIsPushedOutOfTheHubToo) {
  const frc::Pose2d inside{constants::kBlueHubCenter.X() - 0.9_m, constants::kBlueHubCenter.Y(),
                           frc::Rotation2d{45_deg}};
  const frc::Pose2d resolved = ResolveRobotCollisions(inside, kHalfLength, kHalfWidth, RobotObstacles({}));
  for (const Box& box : RobotObstacles({})) {
    // No corner of the bumper may be left inside a structure
    for (double sx : {-1.0, 1.0}) {
      for (double sy : {-1.0, 1.0}) {
        const frc::Translation2d corner =
            resolved.Translation() + At(sx * kHalfLength, sy * kHalfWidth).RotateBy(resolved.Rotation());
        const bool insideBox = corner.X().value() > box.minX + 1e-6 && corner.X().value() < box.maxX - 1e-6 &&
                               corner.Y().value() > box.minY + 1e-6 && corner.Y().value() < box.maxY - 1e-6;
        EXPECT_FALSE(insideBox);
      }
    }
  }
}

TEST(FieldGeometryTest, BumpsAreOnlyWallsWhenTheRobotCannotCrossThem) {
  // Standing on the bump next to the blue hub
  const frc::Pose2d onBump{constants::kBlueHubCenter.X(),
                           constants::kBlueHubCenter.Y() - units::meter_t{constants::kHubHalfSizeMeters + 0.9},
                           frc::Rotation2d{}};
  const frc::Pose2d canCross =
      ResolveRobotCollisions(onBump, kHalfLength, kHalfWidth, RobotObstacles(FieldOptions{true, true}));
  const frc::Pose2d cannotCross =
      ResolveRobotCollisions(onBump, kHalfLength, kHalfWidth, RobotObstacles(FieldOptions{false, true}));
  EXPECT_NEAR(canCross.Translation().Distance(onBump.Translation()).value(), 0.0, 1e-9);
  EXPECT_GT(cannotCross.Translation().Distance(onBump.Translation()).value(), 0.3);
}

TEST(FieldGeometryTest, RobotStaysInsideTheFieldWalls) {
  const frc::Pose2d outside{-1.0_m, 9.0_m, frc::Rotation2d{30_deg}};
  const frc::Pose2d resolved = ResolveRobotCollisions(outside, kHalfLength, kHalfWidth, {});
  EXPECT_GT(resolved.X().value(), kHalfLength - 1e-6);
  EXPECT_LT(resolved.Y().value(), constants::kFieldWidthMeters - kHalfWidth + 1e-6);
}

TEST(FieldGeometryTest, CircleVsBoxPointsOutOfTheBox) {
  const Box box{0.0, 0.0, 1.0, 1.0};
  const auto contact = CircleVsBox(At(1.05, 0.5), 0.1, box);
  ASSERT_TRUE(contact.has_value());
  EXPECT_NEAR(contact->normalX, 1.0, 1e-9);
  EXPECT_NEAR(contact->depth, 0.05, 1e-9);
  EXPECT_FALSE(CircleVsBox(At(1.2, 0.5), 0.1, box).has_value());
}

TEST(BallPhysicsTest, RollingBallSlowsDownAndStops) {
  FieldBalls field;
  field.Add(At(8.0, 2.0), At(1.0, 0.0));
  for (int i = 0; i < 100; ++i) {
    field.Step(0.02, ParkedRobot(frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{}}));
  }
  EXPECT_DOUBLE_EQ(field.Velocities()[0].Norm().value(), 0.0);
  // v^2 / (2a) = 1 / 3 m with 1.5 m/s^2 of carpet friction
  EXPECT_NEAR(field.Positions()[0].X().value(), 8.0 + 1.0 / (2.0 * constants::kBallRollingDecelMps2), 0.03);
}

TEST(BallPhysicsTest, BallBouncesOffTheHub) {
  const double hubFrontX = constants::kBlueHubCenter.X().value() - constants::kHubHalfSizeMeters;
  FieldBalls field;
  field.Add(At(hubFrontX - 0.3, constants::kBlueHubCenter.Y().value()), At(2.0, 0.0));
  for (int i = 0; i < 25; ++i) {
    field.Step(0.02, ParkedRobot(frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{}}));
  }
  EXPECT_LT(field.Positions()[0].X().value(), hubFrontX - kBallRadius + 1e-6) << "ball went into the hub";
  EXPECT_LT(field.Velocities()[0].X().value(), 0.0) << "ball should be rolling back out";
}

TEST(BallPhysicsTest, DrivingRobotPushesBallsAhead) {
  FieldBalls field;
  field.Reset({At(3.0, 2.0)});
  // Robot front bumper overlapping the ball, driving forward at 2 m/s
  const frc::Pose2d robot{units::meter_t{3.0 - kHalfLength}, 2.0_m, frc::Rotation2d{}};
  field.Step(0.02, RobotBody{robot, frc::ChassisSpeeds{2_mps, 0_mps, 0_rad_per_s}, kHalfLength, kHalfWidth});
  EXPECT_GE(field.Positions()[0].X().value(), 3.0 + kBallRadius - 1e-6);
  EXPECT_GT(field.Velocities()[0].X().value(), 1.9) << "ball should leave at least at bumper speed";
}

TEST(BallPhysicsTest, BallsKnockIntoEachOther) {
  FieldBalls field;
  field.Reset({At(8.0, 2.0), At(8.3, 2.0)});
  field.Add(At(7.6, 2.0), At(2.0, 0.0));  // rolling into the first one
  for (int i = 0; i < 50; ++i) {
    field.Step(0.02, ParkedRobot(frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{}}));
  }
  EXPECT_GT(field.Positions()[0].X().value(), 8.05) << "hit ball did not move";
  for (size_t i = 0; i < field.Positions().size(); ++i) {
    for (size_t j = i + 1; j < field.Positions().size(); ++j) {
      EXPECT_GE(field.Positions()[i].Distance(field.Positions()[j]).value(),
                constants::kBallDiameterMeters - 0.01)
          << "balls overlap";
    }
  }
}

TEST(BallPhysicsTest, StartingPileStaysStillWithNothingTouchingIt) {
  FieldBalls field;
  field.Reset(StartingFieldBalls());
  const RobotBody farAway = ParkedRobot(frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{}});
  field.Step(0.02, farAway);  // one settling pass after a reset
  const std::vector<frc::Translation2d> before = field.Positions();
  const unsigned version = field.Version();
  field.Step(0.02, farAway);
  EXPECT_EQ(field.Positions(), before);
  EXPECT_EQ(field.Version(), version) << "telemetry would resend 400 balls every loop";
}

// ---------------- Bumps and trenches ----------------

namespace {
// The middle of the bump below the blue hub (between the hub and the low trench)
frc::Translation2d BlueLowBumpCenter() {
  const double y = constants::kBlueHubCenter.Y().value() - constants::kHubHalfSizeMeters -
                   constants::kBumpLengthMeters / 2.0;
  return At(constants::kBlueHubCenter.X().value(), y);
}
}  // namespace

TEST(FieldGeometryTest, BumpIsARampUpAFlatTopAndARampDown) {
  const frc::Translation2d top = BlueLowBumpCenter();
  const double x = top.X().value();
  const double y = top.Y().value();
  const double edge = constants::kBumpHalfDepthMeters;
  const double ramp = constants::kBumpRampRunMeters;
  EXPECT_NEAR(FloorHeightAt(top), constants::kBumpHeightMeters, 1e-9);
  EXPECT_NEAR(FloorHeightAt(At(x - edge + ramp / 2.0, y)), constants::kBumpHeightMeters / 2.0, 1e-9);
  EXPECT_NEAR(FloorHeightAt(At(x + edge + 0.05, y)), 0.0, 1e-9);  // carpet past the far ramp
  EXPECT_NEAR(FloorHeightAt(At(2.0, 4.0)), 0.0, 1e-9);            // open carpet

  // Going toward +x the near ramp climbs and the far ramp drops
  EXPECT_GT(FloorSlopeAt(At(x - edge + ramp / 2.0, y)).X().value(), 0.2);
  EXPECT_LT(FloorSlopeAt(At(x + edge - ramp / 2.0, y)).X().value(), -0.2);
  EXPECT_NEAR(FloorSlopeAt(top).Norm().value(), 0.0, 1e-9);
}

TEST(FieldGeometryTest, TrenchSupportBetweenBumpAndTrenchIsSolid) {
  // Robot centered on the gap between the low bump's end and the trench lane
  const double hubX = constants::kBlueHubCenter.X().value();
  const double bumpEnd = constants::kBlueHubCenter.Y().value() - constants::kHubHalfSizeMeters -
                         constants::kBumpLengthMeters;
  const double gapMiddle = (bumpEnd + constants::kTrenchWidthMeters) / 2.0;
  const frc::Pose2d inSupport{units::meter_t{hubX}, units::meter_t{gapMiddle}, frc::Rotation2d{}};
  const frc::Pose2d resolved =
      ResolveRobotCollisions(inSupport, kHalfLength, kHalfWidth, RobotObstacles(FieldOptions{true, true}));
  EXPECT_GT(resolved.Translation().Distance(inSupport.Translation()).value(), 0.3)
      << "the trench support should push the robot out";

  // ...and a robot in the middle of the trench lane (under the arm) is free
  const frc::Pose2d inTrench{units::meter_t{hubX}, units::meter_t{constants::kTrenchWidthMeters / 2.0 + 0.05},
                             frc::Rotation2d{}};
  const frc::Pose2d underArm =
      ResolveRobotCollisions(inTrench, kHalfLength, kHalfWidth, RobotObstacles(FieldOptions{true, true}));
  EXPECT_NEAR(underArm.Translation().Distance(inTrench.Translation()).value(), 0.0, 1e-9);
}

TEST(BallPhysicsTest, BallOnARampRollsOffAndStopsOnTheCarpet) {
  const frc::Translation2d top = BlueLowBumpCenter();
  // On the far ramp, halfway down
  const double onRampX = top.X().value() + constants::kBumpHalfDepthMeters - constants::kBumpRampRunMeters / 2.0;
  FieldBalls field;
  field.Add(At(onRampX, top.Y().value()), At(0.0, 0.0));
  for (int i = 0; i < 250; ++i) {
    field.Step(0.02, ParkedRobot(frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{}}));
  }
  const frc::Translation2d end = field.Positions()[0];
  EXPECT_NEAR(FloorHeightAt(end), 0.0, 1e-9) << "ball should have rolled off the bump";
  EXPECT_GT(end.X().value(), top.X().value() + constants::kBumpHalfDepthMeters);
  EXPECT_DOUBLE_EQ(field.Velocities()[0].Norm().value(), 0.0) << "and come to rest on the carpet";
}
