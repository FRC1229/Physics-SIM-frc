#include "sim/FieldBalls.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include "sim/SimConstants.h"

namespace robotsim {

namespace {

constexpr double kMaxSubStepSeconds = 0.005;  // small enough that a 4 m/s robot can't skip over a ball
constexpr double kBallRadius = constants::kBallDiameterMeters / 2.0;

frc::Translation2d Vec(double x, double y) {
  return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
}

// Bounces a velocity off a surface with this outward normal (only if moving into it).
frc::Translation2d Reflect(const frc::Translation2d& velocity, double normalX, double normalY,
                           double restitution) {
  const double intoSurface = velocity.X().value() * normalX + velocity.Y().value() * normalY;
  if (intoSurface >= 0.0) {
    return velocity;  // already moving away
  }
  const double change = -(1.0 + restitution) * intoSurface;
  return Vec(velocity.X().value() + change * normalX, velocity.Y().value() + change * normalY);
}

std::int64_t CellKey(double x, double y) {
  const auto cellX = static_cast<std::int64_t>(std::floor(x / constants::kBallDiameterMeters));
  const auto cellY = static_cast<std::int64_t>(std::floor(y / constants::kBallDiameterMeters));
  return cellX * 100000 + cellY;
}

}  // namespace

FieldBalls::FieldBalls() : m_obstacles(BallObstacles()) {}

void FieldBalls::Reset(std::vector<frc::Translation2d> balls) {
  m_positions = std::move(balls);
  m_velocities.assign(m_positions.size(), frc::Translation2d{});
  m_unsettled = true;
  ++m_version;
}

void FieldBalls::Clear() {
  m_positions.clear();
  m_velocities.clear();
  ++m_version;
}

void FieldBalls::Add(const frc::Translation2d& position, const frc::Translation2d& velocity) {
  const double x = std::clamp(position.X().value(), kBallRadius, constants::kFieldLengthMeters - kBallRadius);
  const double y = std::clamp(position.Y().value(), kBallRadius, constants::kFieldWidthMeters - kBallRadius);
  m_positions.push_back(Vec(x, y));
  m_velocities.push_back(velocity);
  m_unsettled = true;
  ++m_version;
}

int FieldBalls::Collect(const frc::Pose2d& robotPose, const IntakeZone& zone, int maxCount) {
  if (maxCount <= 0) {
    return 0;
  }

  // Every ball inside the zone, as (distance forward, index)
  std::vector<std::pair<double, size_t>> inZone;
  for (size_t i = 0; i < m_positions.size(); ++i) {
    const frc::Translation2d local = (m_positions[i] - robotPose.Translation()).RotateBy(-robotPose.Rotation());
    const double x = local.X().value();
    if (x >= zone.nearX && x <= zone.farX && std::abs(local.Y().value()) <= zone.halfWidth) {
      inZone.emplace_back(std::abs(x), i);  // distance from the robot, either side
    }
  }

  // Closest to the bumper first; erase from the back so the indices stay valid
  std::sort(inZone.begin(), inZone.end());
  const size_t count = std::min(inZone.size(), static_cast<size_t>(maxCount));
  std::vector<size_t> taken;
  taken.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    taken.push_back(inZone[i].second);
  }
  std::sort(taken.rbegin(), taken.rend());
  for (size_t index : taken) {
    m_positions.erase(m_positions.begin() + static_cast<std::ptrdiff_t>(index));
    m_velocities.erase(m_velocities.begin() + static_cast<std::ptrdiff_t>(index));
  }
  if (count > 0) {
    ++m_version;
  }
  return static_cast<int>(count);
}

void FieldBalls::Step(double dtSeconds, const RobotBody& robot) {
  if (dtSeconds <= 0.0 || m_positions.empty()) {
    return;
  }
  const int steps = static_cast<int>(std::ceil(dtSeconds / kMaxSubStepSeconds));
  const double dt = dtSeconds / steps;
  // robot.pose is where the robot is at the END of this step. Walk it there in
  // small pieces so a fast robot sweeps through balls instead of jumping over them.
  for (int i = 1; i <= steps; ++i) {
    const double back = dtSeconds * (1.0 - static_cast<double>(i) / steps);
    RobotBody partial = robot;
    partial.pose = frc::Pose2d{
        robot.pose.X() - units::meter_t{robot.fieldSpeeds.vx.value() * back},
        robot.pose.Y() - units::meter_t{robot.fieldSpeeds.vy.value() * back},
        robot.pose.Rotation() - frc::Rotation2d{units::radian_t{robot.fieldSpeeds.omega.value() * back}}};
    SubStep(dt, partial);
  }
}

void FieldBalls::SubStep(double dt, const RobotBody& robot) {
  const bool moving = Integrate(dt);
  const bool pushed = PushedByRobot(robot);
  // Nothing rolling and nothing touched: the pile is at rest, skip the collision work
  if (!moving && !pushed && !m_unsettled) {
    return;
  }
  BounceOffStructures();
  BounceOffEachOther();
  m_unsettled = false;
  ++m_version;
}

// Rolling with carpet friction; slow enough balls stop. On a ramp gravity rolls
// them downhill (a rolling ball speeds up at 5/7 g sin(angle)). True if any ball moved.
bool FieldBalls::Integrate(double dt) {
  constexpr double kRollingGravity = 5.0 / 7.0 * constants::kGravity;
  bool anyMoving = false;
  for (size_t i = 0; i < m_positions.size(); ++i) {
    frc::Translation2d& velocity = m_velocities[i];
    const frc::Translation2d slope = FloorSlopeAt(m_positions[i]);
    const double steepness = slope.Norm().value();
    // Steep enough that gravity beats the carpet: the ball can't stay put
    const double downhill = kRollingGravity * steepness / std::sqrt(1.0 + steepness * steepness);
    const bool sliding = downhill > constants::kBallRollingDecelMps2;
    if (sliding) {
      velocity = velocity - slope * (downhill * dt / steepness);
    }
    const double speed = velocity.Norm().value();
    if (!sliding && speed < constants::kBallStopSpeedMps) {
      velocity = frc::Translation2d{};
      continue;
    }
    const double slower = std::max(0.0, speed - constants::kBallRollingDecelMps2 * dt);
    velocity = velocity * (slower / speed);
    m_positions[i] = m_positions[i] + velocity * dt;
    anyMoving = true;
  }
  return anyMoving;
}

// The bumpers shove balls out of the way (and give them the bumper's speed).
// True if the robot touched any ball.
bool FieldBalls::PushedByRobot(const RobotBody& robot) {
  bool anyPushed = false;
  const double robotX = robot.pose.X().value();
  const double robotY = robot.pose.Y().value();
  const double vx = robot.fieldSpeeds.vx.value();
  const double vy = robot.fieldSpeeds.vy.value();
  const double omega = robot.fieldSpeeds.omega.value();
  for (size_t i = 0; i < m_positions.size(); ++i) {
    const auto contact = CircleVsRobot(m_positions[i], kBallRadius, robot.pose, robot.halfLength, robot.halfWidth);
    if (!contact.has_value()) {
      continue;
    }
    const double nx = contact->normalX;
    const double ny = contact->normalY;
    m_positions[i] = m_positions[i] + Vec(nx * contact->depth, ny * contact->depth);
    // Speed of the bumper where it touches the ball (driving + spinning)
    const double rx = m_positions[i].X().value() - robotX;
    const double ry = m_positions[i].Y().value() - robotY;
    const frc::Translation2d bumper = Vec(vx - omega * ry, vy + omega * rx);
    const frc::Translation2d relative = Reflect(m_velocities[i] - bumper, nx, ny, constants::kBallWallRestitution);
    m_velocities[i] = relative + bumper;
    anyPushed = true;
  }
  return anyPushed;
}

void FieldBalls::BounceOffStructures() {
  const double maxX = constants::kFieldLengthMeters - kBallRadius;
  const double maxY = constants::kFieldWidthMeters - kBallRadius;
  const double e = constants::kBallWallRestitution;
  for (size_t i = 0; i < m_positions.size(); ++i) {
    for (const Box& box : m_obstacles) {
      if (const auto contact = CircleVsBox(m_positions[i], kBallRadius, box)) {
        m_positions[i] = m_positions[i] + Vec(contact->normalX * contact->depth, contact->normalY * contact->depth);
        m_velocities[i] = Reflect(m_velocities[i], contact->normalX, contact->normalY, e);
      }
    }
    // Field walls
    double x = m_positions[i].X().value();
    double y = m_positions[i].Y().value();
    if (x < kBallRadius) { x = kBallRadius; m_velocities[i] = Reflect(m_velocities[i], 1.0, 0.0, e); }
    if (x > maxX) { x = maxX; m_velocities[i] = Reflect(m_velocities[i], -1.0, 0.0, e); }
    if (y < kBallRadius) { y = kBallRadius; m_velocities[i] = Reflect(m_velocities[i], 0.0, 1.0, e); }
    if (y > maxY) { y = maxY; m_velocities[i] = Reflect(m_velocities[i], 0.0, -1.0, e); }
    m_positions[i] = Vec(x, y);
  }
}

// Equal-mass balls: overlapping pairs are pushed apart and trade speed along the line between them.
void FieldBalls::BounceOffEachOther() {
  std::unordered_map<std::int64_t, std::vector<size_t>> grid;
  grid.reserve(m_positions.size());
  for (size_t i = 0; i < m_positions.size(); ++i) {
    grid[CellKey(m_positions[i].X().value(), m_positions[i].Y().value())].push_back(i);
  }
  const double diameter = constants::kBallDiameterMeters;
  for (size_t i = 0; i < m_positions.size(); ++i) {
    const double xi = m_positions[i].X().value();
    const double yi = m_positions[i].Y().value();
    for (int cx = -1; cx <= 1; ++cx) {
      for (int cy = -1; cy <= 1; ++cy) {
        const auto cell = grid.find(CellKey(xi + cx * diameter, yi + cy * diameter));
        if (cell == grid.end()) {
          continue;
        }
        for (size_t j : cell->second) {
          if (j <= i) {
            continue;  // each pair once
          }
          const frc::Translation2d offset = m_positions[i] - m_positions[j];
          const double distance = offset.Norm().value();
          if (distance >= diameter) {
            continue;
          }
          const double nx = distance > 1e-9 ? offset.X().value() / distance : 1.0;
          const double ny = distance > 1e-9 ? offset.Y().value() / distance : 0.0;
          const double push = (diameter - distance) / 2.0;
          m_positions[i] = m_positions[i] + Vec(nx * push, ny * push);
          m_positions[j] = m_positions[j] - Vec(nx * push, ny * push);
          const frc::Translation2d relative = m_velocities[i] - m_velocities[j];
          const double closing = relative.X().value() * nx + relative.Y().value() * ny;
          if (closing < 0.0) {
            const double impulse = -(1.0 + constants::kBallBallRestitution) * closing / 2.0;
            m_velocities[i] = m_velocities[i] + Vec(nx * impulse, ny * impulse);
            m_velocities[j] = m_velocities[j] - Vec(nx * impulse, ny * impulse);
          }
        }
      }
    }
  }
}

}  // namespace robotsim
