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
