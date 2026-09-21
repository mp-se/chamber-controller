// Host stub for the espframework logger (the real one also pulls in Arduino.h).
#pragma once
#include <Arduino.h>

struct StubLog {
  template <class... A> void verbose(const char*, A...) {}
  template <class... A> void notice(const char*, A...) {}
  template <class... A> void info(const char*, A...) {}
  template <class... A> void warning(const char*, A...) {}
  template <class... A> void error(const char*, A...) {}
};
static StubLog Log;
