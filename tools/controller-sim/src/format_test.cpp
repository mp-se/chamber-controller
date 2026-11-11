// parseBytes() (sensor addresses from the configuration) and roundTo() (values for JSON).
#include <NumberFormats.hpp>
#include <TemperatureFormats.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>

uint32_t g_millis = 0;
uint32_t g_lastDelay = 0;

static int failures = 0;
static void report(const char* name, bool ok) {
  std::printf("  %-70s %s\n", name, ok ? "PASS" : "FAIL");
  if (!ok) failures++;
}

static const uint8_t kAddr[8] = {0x28, 0xFF, 0x64, 0x1E, 0x0F, 0xAB, 0xCD, 0xEF};

// The buffer is followed by canary bytes that must stay untouched
struct Buf {
  uint8_t data[8];
  uint8_t canary[24];
  Buf() {
    std::memset(data, 0x55, sizeof(data));
    std::memset(canary, 0xC3, sizeof(canary));
  }
  bool canaryOk() const {
    for (uint8_t c : canary)
      if (c != 0xC3) return false;
    return true;
  }
  bool isZero() const {
    for (uint8_t d : data)
      if (d != 0) return false;
    return true;
  }
};

int main() {
  std::printf("parseBytes\n");
  {
    Buf b;
    bool r = parseBytes(b.data, "28FF641E0FABCDEF", 8);
    report("16 upper case hex digits", r && !std::memcmp(b.data, kAddr, 8) && b.canaryOk());
  }
  {
    Buf b;
    bool r = parseBytes(b.data, "28ff641e0fabcdef", 8);
    report("16 lower case hex digits", r && !std::memcmp(b.data, kAddr, 8) && b.canaryOk());
  }
  {
    Buf b;
    char text[17];
    printBytes(kAddr, 8, text);
    report("round trip with printBytes", parseBytes(b.data, text, 8) && !std::memcmp(b.data, kAddr, 8));
  }
  {
    Buf b;
    report("odd length '1': rejected, buffer cleared", !parseBytes(b.data, "1", 8) && b.isZero() && b.canaryOk());
  }
  {
    Buf b;
    report("48 hex digits into 8 bytes: rejected, nothing written past 8",
           !parseBytes(b.data, "28FF641E0FABCDEF00112233445566778899AABBCCDDEEFF", 8) && b.isZero() && b.canaryOk());
  }
  {
    Buf b;
    report("18 digits (a trailing byte): rejected", !parseBytes(b.data, "28FF641E0FABCDEFAB", 8) && b.isZero() && b.canaryOk());
  }
  {
    Buf b;
    report("non hex character: rejected", !parseBytes(b.data, "28FF641E0FABCDEG", 8) && b.isZero() && b.canaryOk());
  }
  {
    Buf b;
    report("empty string: rejected", !parseBytes(b.data, "", 8) && b.isZero() && b.canaryOk());
  }
  {
    Buf b;
    report("null string: rejected", !parseBytes(b.data, nullptr, 8) && b.isZero() && b.canaryOk());
  }

  std::printf("roundTo\n");
  report("19.55 to 1 decimal is 19.6", roundTo(19.55, 1) == 19.6);
  report("-3.25 to 1 decimal is -3.3 (half away from zero)", roundTo(-3.25, 1) == -3.3);
  report("1.23456 to 2 decimals is 1.23", roundTo(1.23456, 2) == 1.23);
  report("1.23456 to 3 decimals is 1.235", roundTo(1.23456, 3) == 1.235);
  report("0 and 4 decimals both give 3 decimals", roundTo(1.23456, 0) == 1.235 && roundTo(1.23456, 4) == 1.235);
  report("a whole number stays as it is", roundTo(20, 1) == 20.0 && roundTo(-7, 3) == -7.0);
  report("NAN (no value) gives 0", roundTo(NAN, 1) == 0.0);
  report("a tiny negative number rounds to zero", roundTo(-0.0004, 3) == 0.0);

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
