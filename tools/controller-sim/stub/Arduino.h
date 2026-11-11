// Host stub for the few Arduino things lib/brewpi-pid uses. The clock is virtual: the test or
// simulation sets g_millis, delay() only records its argument in g_lastDelay.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <stdlib.h>
#include <sys/wait.h>

// The host libc declares wait(), the library has a global object called wait. Rename the
// library's after every system header that could declare the function has been included.
#define wait brewpi_wait

#define F(x) x
#define CR "\n"
#define PSTR(x) x
#define strcpy_P strcpy
#define vsnprintf_P vsnprintf

// Arduino-ESP32 provides these, TempControl.cpp expects _min/_max and constrain()
template <class T> T _min(T a, T b) { return a < b ? a : b; }
template <class T> T _max(T a, T b) { return a > b ? a : b; }
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
using std::abs;
using std::isinf;
using std::isnan;
using std::max;
using std::min;

class String {
 public:
  String() {}
  String(const char* s) : _s(s) {}
  String(char c) : _s(1, c) {}
  const char* c_str() const { return _s.c_str(); }
  unsigned length() const { return static_cast<unsigned>(_s.size()); }
  char charAt(unsigned i) const { return i < _s.size() ? _s[i] : 0; }
  // Only the revisions before the floating point rewrite parse temperatures from a String
  void trim() {
    size_t b = _s.find_first_not_of(" \t\r\n");
    size_t e = _s.find_last_not_of(" \t\r\n");
    _s = b == std::string::npos ? std::string() : _s.substr(b, e - b + 1);
  }
  double toDouble() const { return atof(_s.c_str()); }
  std::string _s;
};

extern uint32_t g_millis;     // virtual clock, defined by the test
extern uint32_t g_lastDelay;  // the last delay() argument, defined by the test

inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000u; }
inline void delay(uint32_t ms) { g_lastDelay = ms; }
inline void delayMicroseconds(uint32_t) {}
