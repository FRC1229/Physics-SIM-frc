#include <gtest/gtest.h>

#include "sim/FieldBalls.h"
#include "sim/SimConstants.h"

using namespace robotsim;

namespace {

const IntakeZone kZone{0.4, 0.7, 0.3};  // 0.4 - 0.7 m in front of center, 0.6 m wide

frc::Translation2d At(double x, double y) {
  return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
}

}  // namespace

TEST(FieldBallsTest, CollectsOnlyBallsInsideTheZone) {
  FieldBalls field;
  field.Reset({At(5.5, 4.0),    // in front, inside
               At(5.5, 4.5),    // too far left
               At(4.5, 4.0),    // behind the robot
               At(5.9, 4.0)});  // out of reach
  const frc::Pose2d robot{5.0_m, 4.0_m, frc::Rotation2d{0_deg}};

  EXPECT_EQ(field.Collect(robot, kZone, 10), 1);
  EXPECT_EQ(field.Count(), 3);
}

TEST(FieldBallsTest, ZoneTurnsWithTheRobot) {
  FieldBalls field;
  field.Reset({At(5.0, 4.5)});  // 0.5 m to the field's +y
  const frc::Pose2d facingPlusY{5.0_m, 4.0_m, frc::Rotation2d{90_deg}};
  const frc::Pose2d facingPlusX{5.0_m, 4.0_m, frc::Rotation2d{0_deg}};

  EXPECT_EQ(field.Collect(facingPlusX, kZone, 10), 0);
  EXPECT_EQ(field.Collect(facingPlusY, kZone, 10), 1);
}

TEST(FieldBallsTest, TakesClosestBallsFirstUpToTheLimit) {
  FieldBalls field;
  field.Reset({At(5.65, 4.0), At(5.45, 4.0), At(5.55, 4.1)});
  const frc::Pose2d robot{5.0_m, 4.0_m, frc::Rotation2d{0_deg}};

  EXPECT_EQ(field.Collect(robot, kZone, 2), 2);
  ASSERT_EQ(field.Count(), 1);
  EXPECT_NEAR(field.Positions()[0].X().value(), 5.65, 1e-9) << "the farthest ball should be left";
}

TEST(FieldBallsTest, CollectsNothingWhenFull) {
  FieldBalls field;
  field.Reset({At(5.5, 4.0)});
  EXPECT_EQ(field.Collect(frc::Pose2d{5.0_m, 4.0_m, frc::Rotation2d{}}, kZone, 0), 0);
  EXPECT_EQ(field.Count(), 1);
}

TEST(FieldBallsTest, AddKeepsBallsInsideTheWalls) {
  FieldBalls field;
  field.Add(At(-1.0, 20.0));
  ASSERT_EQ(field.Count(), 1);
  const double r = constants::kBallDiameterMeters / 2.0;
  EXPECT_NEAR(field.Positions()[0].X().value(), r, 1e-9);
  EXPECT_NEAR(field.Positions()[0].Y().value(), constants::kFieldWidthMeters - r, 1e-9);
}

TEST(FieldBallsTest, StartingLayoutIsOnTheFieldAndAwayFromTheHubs) {
  const std::vector<frc::Translation2d> balls = StartingFieldBalls();
  EXPECT_EQ(balls.size(), static_cast<size_t>(constants::kNeutralBallColumns * constants::kNeutralBallRows +
                                              2 * constants::kDepotBallColumns * constants::kDepotBallRows));
  for (const frc::Translation2d& ball : balls) {
    EXPECT_GT(ball.X().value(), 0.0);
    EXPECT_LT(ball.X().value(), constants::kFieldLengthMeters);
    EXPECT_GT(ball.Y().value(), 0.0);
    EXPECT_LT(ball.Y().value(), constants::kFieldWidthMeters);
    EXPECT_GT(ball.Distance(constants::kBlueHubCenter).value(), constants::kHubOuterRadiusMeters);
    EXPECT_GT(ball.Distance(constants::kRedHubCenter).value(), constants::kHubOuterRadiusMeters);
  }
}
