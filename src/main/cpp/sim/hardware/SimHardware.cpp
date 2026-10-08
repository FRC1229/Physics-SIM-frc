#include "sim/hardware/SimHardware.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

namespace robotsim {

// ---------------- REV ----------------

SparkMotor::SparkMotor(rev::spark::SparkBase& spark, const frc::DCMotor& gearbox)
    : m_gearbox(gearbox), m_sim(&spark, &m_gearbox), m_encoder(m_sim.GetRelativeEncoderSim()) {}

// SparkSim::iterate() also integrates its own copy of the position and writes it
// into the encoder. The sim's physics is the only source of truth, so put
// back our position afterwards (otherwise motion is counted twice, and a jump
// like resetting a mechanism is undone on the next loop).
double SparkMotor::Update(double motorRpm, double batteryVolts, double dtSeconds) {
  // The robot code zeroed / set the encoder itself (e.g. ResetEncoders): keep its value
  const double reading = m_encoder.GetPosition();
  if (reading != m_written) {
    m_position = reading;
  }
  m_sim.iterate(motorRpm, batteryVolts, dtSeconds);
  SetPosition(m_position);
  return m_sim.GetAppliedOutput() * batteryVolts;
}

// Written in motor rotations / RPM: assumes the robot code leaves the SPARK's
// encoder conversion factors at 1 (it scales the readings itself).
void SparkMotor::MoveEncoder(double motorRotations, double motorRpm) {
  m_position += motorRotations;
  SetPosition(m_position);
  m_encoder.SetVelocity(motorRpm);
}

void SparkMotor::SetPosition(double motorRotations) {
  m_sim.SetPosition(motorRotations);
  m_encoder.SetPosition(motorRotations);
  m_written = m_encoder.GetPosition();
}

// ---------------- CTRE ----------------

CANcoderSensor::CANcoderSensor(ctre::phoenix6::hardware::CANcoder& cancoder)
    : m_sim(cancoder.GetSimState()) {}

void CANcoderSensor::Set(double rotations, double rotationsPerSecond, double batteryVolts) {
  m_sim.SetSupplyVoltage(units::volt_t{batteryVolts});
  m_sim.SetRawPosition(units::turn_t{rotations});
  m_sim.SetVelocity(units::turns_per_second_t{rotationsPerSecond});
}

Pigeon2Gyro::Pigeon2Gyro(ctre::phoenix6::hardware::Pigeon2& pigeon) : m_sim(pigeon.GetSimState()) {}

void Pigeon2Gyro::AddYaw(double degrees, double degreesPerSecond, double batteryVolts) {
  m_sim.SetSupplyVoltage(units::volt_t{batteryVolts});
  m_sim.AddYaw(units::degree_t{degrees});
  m_sim.SetAngularVelocityZ(units::degrees_per_second_t{degreesPerSecond});
}

void Pigeon2Gyro::SetYaw(double degrees) {
  m_sim.SetRawYaw(units::degree_t{degrees});
}

// Phoenix 6 uses the same right-hand-rule axes as WPILib
void Pigeon2Gyro::SetTilt(double pitchDegrees, double rollDegrees) {
  m_sim.SetPitch(units::degree_t{pitchDegrees});
  m_sim.SetRoll(units::degree_t{rollDegrees});
}

}  // namespace robotsim
