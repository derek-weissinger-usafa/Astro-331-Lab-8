#pragma once
#include <Arduino.h>

// Line-oriented ASCII command parser over any Stream (XBee Serial8 and USB Serial
// both use this). Commands (case-insensitive, newline terminated):
//   ARM            hold level, yaw 0
//   DISARM | KILL  stop all wheels immediately (KILL is also accepted as "K")
//   STATUS         one-line state report
//   PING           reply "PONG"
//   TEST <A-D> <duty>   bench: spin one wheel open-loop at duty (-0.5..0.5) for 1 s (Idle only)
enum class Cmd : uint8_t { None, Arm, Disarm, Kill, Status, Ping, Test, Unknown };

struct Command {
  Cmd cmd = Cmd::None;
  int wheel = -1;    // Test: 0..3 for A..D
  float value = 0;   // Test: duty
};

class CommandLink {
 public:
  explicit CommandLink(Stream& s) : s_(s) {}

  // Drains available bytes. Returns true (and fills `out`) when a full line was parsed.
  // Call repeatedly until it returns false.
  bool poll(Command& out);

  // Non-blocking send: drops the message if the TX buffer lacks room.
  bool send(const char* msg);

 private:
  Stream& s_;
  char buf_[40];
  uint8_t len_ = 0;
  bool overflow_ = false;
};
