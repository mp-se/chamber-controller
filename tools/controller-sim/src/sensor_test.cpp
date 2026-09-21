// TempSensor: filter start on the first reading, and behaviour over a short and a long
// disconnect. The sensor behaves like the BLE beer sensor: init() succeeds but there is no reading
// until data arrives. The object lives in memory poisoned with 0xA5 like uninitialised heap.
#include <TempSensor.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

uint32_t g_millis = 0;
uint32_t g_lastDelay = 0;

struct FakeSensor : BasicTempSensor {
  bool hasData = false;
  temperature value = NAN;
  bool isConnected() const override { return hasData; }
  bool init() override { return true; }
  temperature read() override { return hasData ? value : TEMP_SENSOR_DISCONNECTED; }
  String getName() const override { return "fake"; }
};

static int failures = 0;
static void check(const char* name, bool ok, double got, double want) {
  std::printf("  %-64s got %10.4f, want %10.4f  %s\n", name, got, want, ok ? "PASS" : "FAIL");
  if (!ok) failures++;
}

int main() {
  std::printf("first reading arrives after init(), memory poisoned with 0xA5\n");
  {
    alignas(TempSensor) static unsigned char mem[sizeof(TempSensor)];
    std::memset(mem, 0xA5, sizeof(mem));
    FakeSensor fake;
    TempSensor* s = new (mem) TempSensor(TEMP_SENSOR_TYPE_BEER, &fake);
    s->init();
    fake.hasData = true;
    fake.value = 20.0;
    s->update();
    check("fast filter after the first reading", std::fabs(s->readFastFiltered() - 20.0) < 1e-9, s->readFastFiltered(), 20.0);
    check("slow filter after the first reading", std::fabs(s->readSlowFiltered() - 20.0) < 1e-9, s->readSlowFiltered(), 20.0);
    for (int i = 0; i < 20; i++) s->update();
    check("slow filter after 20 more readings", std::fabs(s->readSlowFiltered() - 20.0) < 1e-6, s->readSlowFiltered(), 20.0);
    check("slope is 0 for a constant input", std::fabs(s->readSlope()) < 1e-6, s->readSlope(), 0.0);
  }

  std::printf("short disconnect (40 + 40 failed reads, never 60 in a row) keeps the filter history\n");
  {
    alignas(TempSensor) static unsigned char mem[sizeof(TempSensor)];
    std::memset(mem, 0xA5, sizeof(mem));
    FakeSensor fake;
    fake.hasData = true;
    fake.value = 20.0;
    TempSensor* s = new (mem) TempSensor(TEMP_SENSOR_TYPE_BEER, &fake);
    s->init();
    for (int round = 0; round < 2; round++) {
      fake.hasData = false;
      for (int i = 0; i < 40; i++) s->update();
      fake.hasData = true;
      fake.value = 25.0;
      s->update();
    }
    check("slow filter still remembers 20, not restarted at 25", s->readSlowFiltered() < 24.5, s->readSlowFiltered(), 24.5);
  }

  std::printf("long disconnect (70 failed reads in a row) restarts the filters at the new reading\n");
  {
    alignas(TempSensor) static unsigned char mem[sizeof(TempSensor)];
    std::memset(mem, 0xA5, sizeof(mem));
    FakeSensor fake;
    fake.hasData = true;
    fake.value = 20.0;
    TempSensor* s = new (mem) TempSensor(TEMP_SENSOR_TYPE_BEER, &fake);
    s->init();
    fake.hasData = false;
    for (int i = 0; i < 70; i++) s->update();
    fake.hasData = true;
    fake.value = 25.0;
    s->update();
    check("fast filter restarted at 25", std::fabs(s->readFastFiltered() - 25.0) < 1e-9, s->readFastFiltered(), 25.0);
    check("slow filter restarted at 25", std::fabs(s->readSlowFiltered() - 25.0) < 1e-9, s->readSlowFiltered(), 25.0);
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
