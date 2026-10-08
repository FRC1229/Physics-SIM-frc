#pragma once

// ================================================================
// The robot's hardware as the simulator sees it, without caring which
// vendor it is. Mechanism sims (swerve, flywheel, arm, ...) only use these
// interfaces, so they work with any robot. Each vendor gets a small adapter:
//
//   SimMotor            <- SparkMotor (REV SPARK MAX / SPARK Flex)
//   SimAbsoluteEncoder  <- CANcoderSensor (CTRE CANcoder)
//   SimGyro             <- Pigeon2Gyro (CTRE Pigeon 2)
//
// New hardware next season (e.g. a TalonFX)? Add one adapter class here.
// ================================================================

#include <memory>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/system/plant/DCMotor.h>
#include <rev/SparkBase.h>
#include <rev/SparkSim.h>
#include <rev/sim/SparkRelativeEncoderSim.h>

namespace robotsim {

// One motor + its controller: what the robot code commands, and what the
// controller's built-in encoder reads back (motor shaft units).
class SimMotor {
 public:
  virtual ~SimMotor() = default;
  // Runs the controller for one step (its own PID etc.) and returns the volts it
  // puts on the motor. motorRpm = how fast the shaft is turning right now.
  virtual double Update(double motorRpm, double batteryVolts, double dtSeconds) = 0;
  // The built-in encoder: the shaft turned this many rotations this step and is
  // now spinning at motorRpm. (Adds to the reading, so robot-code resets stick.)
  virtual void MoveEncoder(double motorRotations, double motorRpm) = 0;
  // Last commanded output, -1..1
  virtual double AppliedOutput() const = 0;
  // The motor model (free speed, stall torque, ...) for the physics
  virtual const frc::DCMotor& Gearbox() const = 0;
};

// An absolute angle sensor (e.g. a swerve module's steering encoder).
class SimAbsoluteEncoder {
 public:
  virtual ~SimAbsoluteEncoder() = default;
  virtual void Set(double rotations, double rotationsPerSecond, double batteryVolts) = 0;
};

// The robot's gyro: yaw from turning, pitch / roll while driving over a bump.
class SimGyro {
 public:
  virtual ~SimGyro() = default;
  virtual void AddYaw(double degrees, double degreesPerSecond, double batteryVolts) = 0;
  virtual void SetYaw(double degrees) = 0;
  // WPILib convention: + pitch = nose down, + roll = left side up
  virtual void SetTilt(double /*pitchDegrees*/, double /*rollDegrees*/) {}
};

// ---------------- REV ----------------

// SPARK MAX or SPARK Flex (any rev::spark::SparkBase) with its built-in encoder.
class SparkMotor final : public SimMotor {
 public:
  SparkMotor(rev::spark::SparkBase& spark, const frc::DCMotor& gearbox);
  SparkMotor(const SparkMotor&) = delete;  // the REV sim keeps a pointer to m_gearbox
  SparkMotor& operator=(const SparkMotor&) = delete;

  double Update(double motorRpm, double batteryVolts, double dtSeconds) override;
  void MoveEncoder(double motorRotations, double motorRpm) override;
  double AppliedOutput() const override { return m_sim.GetAppliedOutput(); }
  const frc::DCMotor& Gearbox() const override { return m_gearbox; }

 private:
  void SetPosition(double motorRotations);

  frc::DCMotor m_gearbox;  // must be declared (constructed) before m_sim
  rev::spark::SparkSim m_sim;
  rev::spark::SparkRelativeEncoderSim m_encoder;
  double m_position = 0.0;  // motor rotations, moved only by the sim's physics (MoveEncoder)
  double m_written = 0.0;   // what we last put in the encoder (anything else = the robot code set it)
};

// ---------------- CTRE ----------------

class CANcoderSensor final : public SimAbsoluteEncoder {
 public:
  explicit CANcoderSensor(ctre::phoenix6::hardware::CANcoder& cancoder);
  void Set(double rotations, double rotationsPerSecond, double batteryVolts) override;

 private:
  ctre::phoenix6::sim::CANcoderSimState& m_sim;
};

class Pigeon2Gyro final : public SimGyro {
 public:
  explicit Pigeon2Gyro(ctre::phoenix6::hardware::Pigeon2& pigeon);
  void AddYaw(double degrees, double degreesPerSecond, double batteryVolts) override;
  void SetYaw(double degrees) override;
  void SetTilt(double pitchDegrees, double rollDegrees) override;

 private:
  ctre::phoenix6::sim::Pigeon2SimState& m_sim;
};

}  // namespace robotsim
