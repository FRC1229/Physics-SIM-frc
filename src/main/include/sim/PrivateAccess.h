#pragma once

// ================================================================
// Lets the simulator read a private member of a robot class (for example
// IndexerSubsystem's feeder motors) WITHOUT changing the robot code.
//
// Standard C++ allows an explicit template instantiation to name a private
// member; the instantiation stores a pointer-to-member that Get() returns.
//
// Usage:
//   struct KickerTag { using type = rev::spark::SparkMax IndexerSubsystem::*; friend type Get(KickerTag); };
//   template struct robotsim::PrivateMember<KickerTag, &IndexerSubsystem::m_kickerMotor>;
//   rev::spark::SparkMax& kicker = indexer.*Get(KickerTag{});
// ================================================================

namespace robotsim {

template <typename Tag, typename Tag::type Member>
struct PrivateMember {
  friend typename Tag::type Get(Tag) { return Member; }
};

}  // namespace robotsim
