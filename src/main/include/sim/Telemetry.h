#pragma once

// ================================================================
// NetworkTables settings for the sim's telemetry.
// ================================================================

#include <networktables/NetworkTableInstance.h>

namespace robotsim {

// NetworkTables only sends a value every 0.1 s by default, so things that move
// (flying balls, the robot) would jump in AdvantageScope 5 sim steps at a time.
// Send these every 20 ms loop instead, and keep every update.
inline nt::PubSubOptions EveryLoop() {
  nt::PubSubOptions options;
  options.periodic = 0.02;
  options.sendAll = true;
  return options;
}

}  // namespace robotsim
