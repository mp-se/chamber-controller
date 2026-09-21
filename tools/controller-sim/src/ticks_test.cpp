// HardwareTicks::secondsSince() (elapsed time from a millis() timestamp, must survive both the
// 18 h 12 m point where 16 bit seconds used to wrap and the millis() rollover) and
// HardwareDelay::seconds().
#include <Ticks.hpp>

#include <cstdio>

uint32_t g_millis = 0;
uint32_t g_lastDelay = 0;

struct Case {
  const char* name;
  uint32_t startMs;    // when the timestamp was taken
  uint32_t nowMs;      // when we ask how long ago that was
  uint32_t expectSec;  // the true answer
};

int main() {
  const Case cases[] = {
      {"cooled 5 s ago", 1000000u, 1005000u, 5},
      {"never cooled since boot (timestamp 0), uptime 5 h", 0u, 18000000u, 18000},
      {"same, uptime 18 h 12 m 21 s (just past 65536 s)", 0u, 65541000u, 65541},
      {"same, uptime 25 h", 0u, 90000000u, 90000},
      {"idle for 19 h 27 m", 1000000u, 1000000u + 70020000u, 70020},
      {"idle for exactly 65536 s", 5000u, 5000u + 65536000u, 65536},
      {"under a second (1999 ms)", 100u, 2099u, 1},
      {"millis() rollover, 9095 ms across it", 0xFFFFF000u, 5000u, 9},
      {"rollover, timestamp 1 ms before it", 0xFFFFFFFFu, 999u, 1},
  };

  int failures = 0;
  std::printf("secondsSince\n");
  for (const Case& c : cases) {
    g_millis = c.startMs;
    ticks_millis_t timestamp = ticks.millis();
    g_millis = c.nowMs;
    uint32_t got = ticks.secondsSince(timestamp);
    bool ok = got == c.expectSec;
    if (!ok) failures++;
    std::printf("  %-52s want %6u got %6u  %s\n", c.name, c.expectSec, got, ok ? "PASS" : "FAIL");
  }

  std::printf("wait.seconds(n) -> delay(ms)\n");
  const struct {
    uint16_t seconds;
    uint32_t expectMs;
  } delays[] = {{2, 2000}, {70, 70000}};
  for (const auto& d : delays) {
    g_lastDelay = 0;
    wait.seconds(d.seconds);
    bool ok = g_lastDelay == d.expectMs;
    if (!ok) failures++;
    std::printf("  wait.seconds(%3u)                                     want %6u got %6u  %s\n", d.seconds, d.expectMs, g_lastDelay, ok ? "PASS" : "FAIL");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
