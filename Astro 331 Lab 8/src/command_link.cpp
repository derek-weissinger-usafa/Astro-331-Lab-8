#include "command_link.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

static Command parseLine(char* line) {
  Command c;
  // Uppercase in place and split into the first token and the rest.
  for (char* p = line; *p; p++) *p = (char)toupper((unsigned char)*p);
  char* save = nullptr;
  char* tok = strtok_r(line, " \t,", &save);
  if (!tok) return c;  // empty line

  if (!strcmp(tok, "ARM")) {
    c.cmd = Cmd::Arm;
  } else if (!strcmp(tok, "DISARM")) {
    c.cmd = Cmd::Disarm;
  } else if (!strcmp(tok, "KILL") || !strcmp(tok, "K")) {
    c.cmd = Cmd::Kill;
  } else if (!strcmp(tok, "STATUS")) {
    c.cmd = Cmd::Status;
  } else if (!strcmp(tok, "PING")) {
    c.cmd = Cmd::Ping;
  } else if (!strcmp(tok, "COUNTS")) {
    c.cmd = Cmd::Counts;
  } else if (!strcmp(tok, "ZERO")) {
    c.cmd = Cmd::Zero;
  } else if (!strcmp(tok, "CALIBRATE")) {
    c.cmd = Cmd::Calibrate;
    char* v = strtok_r(nullptr, " \t,", &save);
    if (v) {
      c.value = (float)atof(v);
      c.hasValue = true;
    }
  } else if (!strcmp(tok, "STOP")) {
    c.cmd = Cmd::Stop;
  } else if (!strcmp(tok, "MAG")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && !strcmp(a, "MAP"))    ? Cmd::MagMap
            : (a && !strcmp(a, "TEST")) ? Cmd::MagTest
            : (a && !strcmp(a, "ON"))   ? Cmd::MagAidOn
            : (a && !strcmp(a, "OFF"))  ? Cmd::MagAidOff
                                        : Cmd::Unknown;
    char* v = strtok_r(nullptr, " \t,", &save);
    if (c.cmd == Cmd::MagTest && v) {
      c.value = (float)atof(v);
      c.hasValue = true;
    }
  } else if (!strcmp(tok, "SLEW")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    if (a && !strcmp(a, "TEST")) {
      c.cmd = Cmd::SlewTest;
    } else if (a && !strcmp(a, "YAW")) {
      c.cmd = Cmd::SlewYaw;
    } else if (a && !strcmp(a, "LIST")) {
      c.cmd = Cmd::SlewList;
    } else if (a && !strcmp(a, "SET")) {
      char* n = strtok_r(nullptr, " \t,", &save);
      char* r = strtok_r(nullptr, " \t,", &save);
      char* p = strtok_r(nullptr, " \t,", &save);
      char* y = strtok_r(nullptr, " \t,", &save);
      if (n && r && p && y) {
        c.cmd = Cmd::SlewSet;
        c.wheel = atoi(n) - 1;
        c.v[0] = (float)atof(r);
        c.v[1] = (float)atof(p);
        c.v[2] = (float)atof(y);
      } else {
        c.cmd = Cmd::Unknown;
      }
    } else {
      c.cmd = Cmd::Unknown;
    }
  } else if (!strcmp(tok, "FAULT")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    char* w = strtok_r(nullptr, " \t,", &save);
    char* d = strtok_r(nullptr, " \t,", &save);
    if (a && !strcmp(a, "CLEAR")) {
      c.cmd = Cmd::FaultClear;
    } else if (a && !strcmp(a, "TEST") && w && !strcmp(w, "REAL")) {
      c.cmd = Cmd::FaultTest;
      c.wheel = -1;
    } else if (a && !strcmp(a, "TEST") && w && w[0] >= 'A' && w[0] <= 'D' && w[1] == '\0') {
      c.cmd = Cmd::FaultTest;
      c.wheel = w[0] - 'A';
      if (d) {
        c.value = (float)atof(d);
        c.hasValue = true;
      }
    } else {
      c.cmd = Cmd::Unknown;
    }
  } else if (!strcmp(tok, "LEVEL")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && !strcmp(a, "ZERO")) ? Cmd::LevelZero : Cmd::Unknown;
  } else if (!strcmp(tok, "STEP")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    char* v = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && !strcmp(a, "TEST")) ? Cmd::StepTest : Cmd::Unknown;
    if (v) {
      c.value = (float)atof(v);
      c.hasValue = true;
    }
  } else if (!strcmp(tok, "SET") || !strcmp(tok, "GET")) {
    const bool isSet = tok[0] == 'S';
    char* n = strtok_r(nullptr, " \t,", &save);
    char* v = strtok_r(nullptr, " \t,", &save);
    if (isSet && (!n || !v)) {
      c.cmd = Cmd::Unknown;
    } else {
      c.cmd = isSet ? Cmd::Set : Cmd::Get;
      if (n) strncpy(c.name, n, sizeof(c.name) - 1);
      if (v) {
        c.value = (float)atof(v);
        c.hasValue = true;
      }
    }
  } else if (!strcmp(tok, "BIAS")) {
    char* v = strtok_r(nullptr, " \t,", &save);
    c.cmd = v ? Cmd::Bias : Cmd::Unknown;
    if (v) c.value = (float)atof(v);
  } else if (!strcmp(tok, "KICK")) {
    char* v = strtok_r(nullptr, " \t,", &save);
    c.cmd = v ? Cmd::Kick : Cmd::Unknown;
    if (v) c.value = (float)atof(v);
  } else if (!strcmp(tok, "WHEEL")) {
    char* w = strtok_r(nullptr, " \t,", &save);
    char* a = strtok_r(nullptr, " \t,", &save);
    const bool on = a && !strcmp(a, "ON"), off = a && !strcmp(a, "OFF");
    if (w && w[0] >= 'A' && w[0] <= 'D' && w[1] == '\0' && (on || off)) {
      c.cmd = Cmd::Wheel;
      c.wheel = w[0] - 'A';
      c.value = on ? 1.0f : 0.0f;
    } else {
      c.cmd = Cmd::Unknown;
    }
  } else if (!strcmp(tok, "RAW")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && !strcmp(a, "ON")) ? Cmd::RawOn : (a && !strcmp(a, "OFF")) ? Cmd::RawOff : Cmd::Unknown;
  } else if (!strcmp(tok, "DEMO")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && !strcmp(a, "SPIN"))    ? Cmd::DemoSpin
            : (a && !strcmp(a, "LEVEL")) ? Cmd::DemoLevel
                                         : Cmd::Unknown;
  } else if (!strcmp(tok, "BEGIN")) {
    char* a = strtok_r(nullptr, " \t,", &save);
    char* b = strtok_r(nullptr, " \t,", &save);
    c.cmd = (a && b && !strcmp(a, "STATIC") && !strcmp(b, "TEST")) ? Cmd::BeginStatic : Cmd::Unknown;
    char* v = strtok_r(nullptr, " \t,", &save);
    if (c.cmd == Cmd::BeginStatic && v) {
      c.value = (float)atof(v);
      c.hasValue = true;
    }
  } else if (!strcmp(tok, "TEST")) {
    char* w = strtok_r(nullptr, " \t,", &save);
    char* d = strtok_r(nullptr, " \t,", &save);
    if (w && d && w[0] >= 'A' && w[0] <= 'D' && w[1] == '\0') {
      c.cmd = Cmd::Test;
      c.wheel = w[0] - 'A';
      c.value = (float)atof(d);
    } else {
      c.cmd = Cmd::Unknown;
    }
  } else if (!strcmp(tok, "SPEED")) {
    char* w = strtok_r(nullptr, " \t,", &save);
    char* v = strtok_r(nullptr, " \t,", &save);
    if (w && v && !strcmp(w, "ALL")) {
      c.wheel = Command::kAllWheels;
    } else if (w && v && w[0] >= 'A' && w[0] <= 'D' && w[1] == '\0') {
      c.wheel = w[0] - 'A';
    }
    if (c.wheel >= 0) {
      c.cmd = Cmd::Speed;
      c.value = (float)atof(v);
    } else {
      c.cmd = Cmd::Unknown;
    }
  } else {
    c.cmd = Cmd::Unknown;
  }
  return c;
}

bool CommandLink::poll(Command& out) {
  while (s_.available() > 0) {
    const int ch = s_.read();
    if (ch < 0) break;
    if (ch == '\n' || ch == '\r') {
      const bool ok = !overflow_ && len_ > 0;
      buf_[len_] = '\0';
      const uint8_t had = len_;
      len_ = 0;
      overflow_ = false;
      if (!ok) {
        if (had == 0) continue;  // blank line / CRLF second byte
        out = Command();
        out.cmd = Cmd::Unknown;
        return true;
      }
      out = parseLine(buf_);
      if (out.cmd == Cmd::None) continue;
      return true;
    }
    if (len_ < sizeof(buf_) - 1) {
      buf_[len_++] = (char)ch;
    } else {
      overflow_ = true;  // too long: discard until newline
    }
  }
  return false;
}

bool CommandLink::send(const char* msg) {
  const size_t n = strlen(msg);
  if ((size_t)s_.availableForWrite() < n) return false;
  s_.write((const uint8_t*)msg, n);
  return true;
}
