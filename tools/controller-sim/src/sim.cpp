// Closed loop simulation of the lib/brewpi-pid controller against a thermal model of a fridge and
// a beer. The same source builds against a library that uses fixed point temperatures (git
// revisions before the floating point rewrite) and one that uses doubles (-DTEMP_IS_DOUBLE), only
// the adapters below differ. See README.md.
//
// usage: sim <traceDir|-> [perturbation [record <dir> | replay <dir>]]
//   traceDir      write a CSV trace per scenario there ('-' = no trace)
//   perturbation  added to the start temperature of fridge and beer, in Celsius
//   record <dir>  keep every sensor reading the controller saw
//   replay <dir>  feed recorded sensor readings into the controller instead of simulating them
// The summary metrics of each scenario are printed on stdout. The controller keeps state between
// runs (PID integral, a static counter in updatePID), so run ONE scenario per process:
// SIM_ONLY=<name>.
#include <ActuatorValue.hpp>
#include <TempControl.hpp>
#include <TempSensor.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

uint32_t g_millis = 0;
uint32_t g_lastDelay = 0;

// ---- adapters between degrees Celsius and the library's `temperature` -----------------------
#ifdef TEMP_IS_DOUBLE
static temperature toTemp(double c) { return c; }
static double fromTemp(temperature t) { return t; }
static double diffFromTemp(temperature t) { return t; }
#else
static temperature toTemp(double c) { return doubleToTemp(c); }
static double fromTemp(temperature t) { return tempToDouble(t, 3); }
static double diffFromTemp(temperature t) { return tempDiffToDouble(t, 3); }
#endif

// ---- a DS18B20 like sensor: 1/16 degree steps, optional deterministic noise -------------------
struct SimSensor : BasicTempSensor {
  const double* value = nullptr;
  bool connected = true;
  double noise = 0;
  uint32_t rng = 1;
  double lastRead = 0;
  // record: every value read() returns is kept (NAN = disconnected). replay: read() returns
  // the recorded values in order, so the controller sees exactly the same sensor input.
  std::vector<double> recorded;
  const std::vector<double>* replay = nullptr;
  size_t pos = 0;
  double lastReplay = 0;

  bool isConnected() const override { return replay ? !std::isnan(lastReplay) : connected; }
  // live: connected. replay: the last value read was a real reading (nothing read yet counts as
  // connected), which is what "connected" was at that moment when it was recorded
  bool init() override {
    if (!replay) return connected;
    return pos == 0 || !std::isnan(lastReplay);
  }
  temperature read() override {
    double v = NAN;
    if (replay) {
      v = pos < replay->size() ? (*replay)[pos++] : NAN;
      lastReplay = v;
      lastRead = v;
    } else if (connected) {
      rng = rng * 1664525u + 1013904223u;
      double n = noise * (((rng >> 8) / 16777216.0) * 2.0 - 1.0);
      lastRead = v = std::floor((*value + n) * 16.0 + 0.5) / 16.0;
    }
    if (!replay) recorded.push_back(v);
    return std::isnan(v) ? TEMP_SENSOR_DISCONNECTED : toTemp(v);
  }
  String getName() const override { return "sim"; }
};

static const char* g_recordDir = nullptr;
static const char* g_replayDir = nullptr;

static std::vector<double> readValues(const std::string& path) {
  std::vector<double> v;
  FILE* f = std::fopen(path.c_str(), "r");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", path.c_str());
    std::exit(2);
  }
  char word[64];
  while (std::fscanf(f, "%63s", word) == 1) v.push_back(std::strcmp(word, "nan") == 0 ? NAN : std::atof(word));
  std::fclose(f);
  return v;
}

static void writeValues(const std::string& path, const std::vector<double>& v) {
  FILE* f = std::fopen(path.c_str(), "w");
  for (double x : v) {
    if (std::isnan(x))
      std::fprintf(f, "nan\n");
    else
      std::fprintf(f, "%.17g\n", x);
  }
  std::fclose(f);
}

struct Scenario {
  const char* name;
  char mode;             // ControllerMode
  double target;         // Celsius
  double ambient;
  double start;          // start temperature of fridge and beer
  double ferment;        // beer self heating, C/s
  double noise;          // sensor noise amplitude, C
  int hours;
  int stepAtHour;        // change the target at this hour (-1 = never)
  double stepTarget;
  int disconnectAtMin;   // fridge sensor disconnected from this minute (-1 = never)
  int disconnectForMin;
  double coolRate;       // C/s the fridge air cools while the compressor runs
  double heatRate;       // C/s the fridge air heats while the heater runs
  int without;           // parts of the device that are not set up, like disabled in the config (see below)
  int beerLostAtMin = 0;   // the beer probe gives no readings from this minute, like a BLE sensor out of range
  int beerLostForMin = 0;  // for this many minutes (0 = never)
};

// A part that is not set up is left as the default of TempControl::init(): a default actuator does
// nothing, a default sensor is never connected.
enum Without { NO_HEATER = 1, NO_COOLER = 2, NO_FRIDGE_PROBE = 4, NO_BEER_PROBE = 8 };

// a small chamber: compressor about 1 C/min, heater about 0.5 C/min
static const Scenario kScenarios[] = {
    {"fridge_cool", ControllerMode::fridgeConstant, 10, 22, 20, 0, 0, 6, -1, 0, -1, 0, 0.016, 0.008, 0},
    {"beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, 0},
    {"beer_heat", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0.008, 0},
    {"beer_step", ControllerMode::beerConstant, 15, 20, 20, 0.00003, 0, 24, 8, 19, -1, 0, 0.016, 0.008, 0},
    {"beer_cool_noisy", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0.03, 24, -1, 0, -1, 0, 0.016, 0.008, 0},
    {"fridge_disconnect", ControllerMode::fridgeConstant, 8, 22, 18, 0, 0, 6, -1, 0, 120, 12, 0.016, 0.008, 0},
    {"beer_heat_noisy", ControllerMode::beerConstant, 20, 8, 12, 0.00002, 0.03, 24, -1, 0, -1, 0, 0.016, 0.008, 0},
    // the same chamber without a heater (nh_ = no heater), the heating actuator stays the default one
    {"nh_fridge_cool", ControllerMode::fridgeConstant, 10, 22, 20, 0, 0, 6, -1, 0, -1, 0, 0.016, 0, NO_HEATER},
    {"nh_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0, NO_HEATER},
    {"nh_beer_cool_noisy", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0.03, 24, -1, 0, -1, 0, 0.016, 0, NO_HEATER},
    // the target is above what the ambient gives and there is nothing to heat with
    {"nh_beer_warm", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0, NO_HEATER},
    // the room is colder than the target, the beer is cooled down to the target and then drifts below it
    {"nh_beer_cold_room", ControllerMode::beerConstant, 12, 8, 14, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0, NO_HEATER},
    {"nh_beer_step", ControllerMode::beerConstant, 15, 20, 20, 0.00003, 0, 24, 8, 19, -1, 0, 0.016, 0, NO_HEATER},
    {"nh_fridge_disconnect", ControllerMode::fridgeConstant, 8, 22, 18, 0, 0, 6, -1, 0, 120, 12, 0.016, 0, NO_HEATER},
    // the beer probe (a BLE sensor) stops updating for hours: the controller has to keep the beer near the
    // target with the fridge probe until it is back. boot: the sensor is not heard for the first 4 hours.
    {"ble_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, 0, 480, 360},
    {"ble_beer_heat", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0.008, 0, 480, 360},
    {"ble_beer_boot", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, 0, 0, 240},
    {"nh_ble_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0, NO_HEATER, 480, 360},
    {"nc_ble_beer_heat", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_COOLER, 480, 360},
    // heater only (nc_ = no cooler): a cold room, the chamber is warmed
    {"nc_fridge_heat", ControllerMode::fridgeConstant, 18, 8, 10, 0, 0, 6, -1, 0, -1, 0, 0.016, 0.008, NO_COOLER},
    {"nc_beer_heat", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_COOLER},
    {"nc_beer_heat_noisy", ControllerMode::beerConstant, 20, 8, 12, 0.00002, 0.03, 24, -1, 0, -1, 0, 0.016, 0.008, NO_COOLER},
    // the target is below the ambient and there is nothing to cool with
    {"nc_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_COOLER},
    {"nc_beer_step", ControllerMode::beerConstant, 15, 8, 10, 0.00003, 0, 24, 8, 19, -1, 0, 0.016, 0.008, NO_COOLER},
    {"nc_fridge_disconnect", ControllerMode::fridgeConstant, 18, 8, 12, 0, 0, 6, -1, 0, 120, 12, 0.016, 0.008, NO_COOLER},
    // one probe only, heater and cooler (fp_ = fridge probe only, bp_ = beer probe only)
    {"fp_fridge_cool", ControllerMode::fridgeConstant, 10, 22, 20, 0, 0, 6, -1, 0, -1, 0, 0.016, 0.008, NO_BEER_PROBE},
    {"fp_fridge_heat", ControllerMode::fridgeConstant, 18, 8, 10, 0, 0, 6, -1, 0, -1, 0, 0.016, 0.008, NO_BEER_PROBE},
    {"fp_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_BEER_PROBE},
    {"bp_beer_cool", ControllerMode::beerConstant, 12, 22, 20, 0.00004, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_FRIDGE_PROBE},
    {"bp_beer_heat", ControllerMode::beerConstant, 18, 10, 10, 0.00002, 0, 24, -1, 0, -1, 0, 0.016, 0.008, NO_FRIDGE_PROBE},
    {"bp_fridge_cool", ControllerMode::fridgeConstant, 10, 22, 20, 0, 0, 6, -1, 0, -1, 0, 0.016, 0.008, NO_FRIDGE_PROBE},
};

struct Sample {
  double t, Tf, Tb, rF, rB, fF, sB, slope, fridgeSetting, beerSetting;
  int state, cool, heat, wait;
  double p, i, d, integral, beerDiff, estPeak, heatEst, coolEst;
  double fsP;  // fridge setting with more decimals than the getter gives
  double fB;   // beer, fast filter
  double epP;  // estimated peak with more decimals than the JSON gives
};

static void run(const Scenario& s, const char* traceFile, double perturb) {
  g_millis = 5000;  // boot time

  double Tf = s.start + perturb, Tb = s.start + perturb;  // model state
  const double Ta = s.ambient;

  tempControl.init(MIN_TIMES_DEFAULT);

  SimSensor fridgeSim, beerSim;
  fridgeSim.value = &Tf;
  beerSim.value = &Tb;
  fridgeSim.noise = beerSim.noise = s.noise;
  fridgeSim.rng = 11;
  beerSim.rng = 23;

  std::vector<double> replayF, replayB;
  if (g_replayDir) {
    replayF = readValues(std::string(g_replayDir) + "/" + s.name + ".F.txt");
    replayB = readValues(std::string(g_replayDir) + "/" + s.name + ".B.txt");
    fridgeSim.replay = &replayF;
    beerSim.replay = &replayB;
  }

  TempSensor fridgeSensor(TEMP_SENSOR_TYPE_FRIDGE, &fridgeSim);
  TempSensor beerSensor(TEMP_SENSOR_TYPE_BEER, &beerSim);
  fridgeSensor.init();
  beerSensor.init();
  if (!(s.without & NO_FRIDGE_PROBE)) tempControl.setFridgeSensor(&fridgeSensor);
  if (!(s.without & NO_BEER_PROBE)) tempControl.setBeerSensor(&beerSensor);

  ValueActuator cooler, heater;
  if (!(s.without & NO_COOLER)) tempControl.setCoolingActuator(&cooler);
  if (!(s.without & NO_HEATER)) tempControl.setHeatingActuator(&heater);

  // the same order as configureTempControl() in main.cpp
  if (s.mode == ControllerMode::fridgeConstant)
    tempControl.setFridgeTargetTemperature(s.target);
  else
    tempControl.setBeerTargetTemperature(s.target);
  tempControl.setMode(s.mode);

  double target = s.target;
  const int steps = s.hours * 3600;
  std::vector<Sample> trace;
  trace.reserve(steps / 10 + 1);

  // metrics
  int coolSwitches = 0, heatSwitches = 0;
  long coolSeconds = 0, heatSeconds = 0;
  bool wasCool = false, wasHeat = false;
  int stateSeconds[9] = {0};
  double firstOnTarget = -1;
  double devSum = 0;
  long devN = 0;
  double lostDev = 0;  // largest beer deviation in the last hour that the beer probe was lost
  bool reached = false;
  double crossFrom = 0;

  for (int k = 0; k < steps; k++) {
    g_millis = 5000 + 1000u * static_cast<uint32_t>(k);

    if (s.stepAtHour >= 0 && k == s.stepAtHour * 3600) {
      target = s.stepTarget;
      if (s.mode == ControllerMode::fridgeConstant)
        tempControl.setFridgeTargetTemperature(target);
      else
        tempControl.setBeerTargetTemperature(target);
      reached = false;
      firstOnTarget = -1;  // measure the time to reach the new target
      crossFrom = k;
    }
    {
      int m = k / 60;
      beerSim.connected = !(m >= s.beerLostAtMin && m < s.beerLostAtMin + s.beerLostForMin);
      if (s.beerLostForMin > 0 && m >= s.beerLostAtMin + s.beerLostForMin - 60 && m < s.beerLostAtMin + s.beerLostForMin)
        lostDev = std::max(lostDev, std::fabs(Tb - target));
    }
    if (s.disconnectAtMin >= 0) {
      int m = k / 60;
      fridgeSim.connected = !(m >= s.disconnectAtMin && m < s.disconnectAtMin + s.disconnectForMin);
    }

    // physics, one second, with the actuator state chosen by the last control decision
    double q = 0;
    if (cooler.isActive()) q -= s.coolRate;
    if (heater.isActive()) q += s.heatRate;
    Tf += q + (Ta - Tf) / 3000.0 + (Tb - Tf) / 2500.0;
    Tb += (Tf - Tb) / 1800.0 + s.ferment;

    tempControl.loop();

    bool c = cooler.isActive(), h = heater.isActive();
    if (c && !wasCool) coolSwitches++;
    if (h && !wasHeat) heatSwitches++;
    wasCool = c;
    wasHeat = h;
    coolSeconds += c;
    heatSeconds += h;
    int st = tempControl.getState();
    if (st >= 0 && st < 9) stateSeconds[st]++;

    // what the controller regulates is what its probe measures: with a single probe that is the probe
    // in the fridge or in the beer, whatever the mode is called
    bool fridgeMode = s.mode == ControllerMode::fridgeConstant;
    double controlled = fridgeMode ? ((s.without & NO_FRIDGE_PROBE) ? Tb : Tf) : ((s.without & NO_BEER_PROBE) ? Tf : Tb);
    if (!reached && std::fabs(controlled - target) < 0.3) {
      reached = true;
      firstOnTarget = (k - crossFrom) / 60.0;
    }
    if (k >= steps / 2 && (s.stepAtHour < 0 || k >= (s.stepAtHour + 4) * 3600)) {
      devSum += std::fabs(controlled - target);
      devN++;
    }

    if (k % 10 == 0) {
      Sample sm;
      sm.t = k;
      sm.Tf = Tf;
      sm.Tb = Tb;
      sm.rF = fridgeSim.lastRead;
      sm.rB = beerSim.lastRead;
      sm.fF = fromTemp(fridgeSensor.readFastFiltered());
      sm.sB = fromTemp(beerSensor.readSlowFiltered());
      sm.slope = diffFromTemp(beerSensor.readSlope());
      sm.fridgeSetting = tempControl.getFridgeTemperatureSetting();
      sm.beerSetting = tempControl.getBeerTemperatureSetting();
      sm.state = st;
      sm.cool = c;
      sm.heat = h;
      sm.wait = tempControl.getWaitTime();

      JsonDocument cv;
      JsonObject cvo = cv.to<JsonObject>();
      tempControl.getControlVariables().toJsonReadable(cvo);
      sm.p = cv.record["p"];
      sm.i = cv.record["i"];
      sm.d = cv.record["d"];
      sm.integral = cv.record["diff_integral"];
      sm.beerDiff = cv.record["beer_diff"];
      sm.estPeak = cv.record["est_peak"];
      JsonDocument cs;
      JsonObject cso = cs.to<JsonObject>();
      tempControl.getControlSettings().toJsonReadable(cso);
      sm.heatEst = cs.record["heat_estimator"];
      sm.coolEst = cs.record["cool_estimator"];
      sm.fsP = fromTemp(tempControl.getControlSettings().fridgeSetting);
      sm.fB = fromTemp(beerSensor.readFastFiltered());
      sm.epP = fromTemp(tempControl.getControlVariables().estimatedPeak);
      trace.push_back(sm);
    }
  }

  if (g_recordDir) {
    writeValues(std::string(g_recordDir) + "/" + s.name + ".F.txt", fridgeSim.recorded);
    writeValues(std::string(g_recordDir) + "/" + s.name + ".B.txt", beerSim.recorded);
  }

  if (traceFile) {
    FILE* f = std::fopen(traceFile, "w");
    std::fprintf(f, "t,Tf,Tb,rF,rB,fF,sB,slope,fridgeSetting,beerSetting,state,cool,heat,wait,p,i,d,integral,beerDiff,estPeak,heatEst,coolEst,fsP,fB,epP\n");
    for (const Sample& m : trace)
      std::fprintf(f, "%.0f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f\n", m.t, m.Tf, m.Tb, m.rF, m.rB, m.fF, m.sB,
                   m.slope, m.fridgeSetting, m.beerSetting, m.state, m.cool, m.heat, m.wait, m.p, m.i, m.d, m.integral, m.beerDiff, m.estPeak, m.heatEst, m.coolEst, m.fsP, m.fB, m.epP);
    std::fclose(f);
  }

  const Sample& last = trace.back();
  std::printf("%-18s coolSw=%3d heatSw=%3d cool%%=%5.1f heat%%=%5.1f reachMin=%6.1f meanDev=%6.3f lostDev=%6.3f  heatEst=%7.4f coolEst=%7.4f  states(s):", s.name, coolSwitches, heatSwitches,
              100.0 * coolSeconds / steps, 100.0 * heatSeconds / steps, firstOnTarget, devN ? devSum / devN : 0.0, lostDev, last.heatEst, last.coolEst);
  for (int i = 0; i < 9; i++) std::printf(" %d", stateSeconds[i]);
  std::printf("\n");
}

int main(int argc, char** argv) {
  const char* dir = argc > 1 ? argv[1] : "-";
  double perturb = argc > 2 ? std::atof(argv[2]) : 0.0;
  if (argc > 4 && std::string(argv[3]) == "record") g_recordDir = argv[4];
  if (argc > 4 && std::string(argv[3]) == "replay") g_replayDir = argv[4];

  const char* only = std::getenv("SIM_ONLY");
  for (const Scenario& s : kScenarios) {
    if (only && std::string(only) != s.name) continue;
    std::string file;
    if (std::string(dir) != "-") file = std::string(dir) + "/" + s.name + ".csv";
    run(s, file.empty() ? nullptr : file.c_str(), perturb);
  }
  return 0;
}
