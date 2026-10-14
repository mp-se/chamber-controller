// The BrewGraph poll without the HTTP part (src/chamberreport.cpp): the request body, reading the reply and
// deciding what to do with it. Built against the real ArduinoJson, see run.sh.
#include <chamberreport.hpp>

#include <cmath>
#include <cstdio>
#include <string>

static int failures = 0;
static void report(const char* name, bool ok) {
  std::printf("  %-78s %s\n", name, ok ? "PASS" : "FAIL");
  if (!ok) failures++;
}

static ChamberReport makeReport(float beer, float fridge, char mode, float target) {
  ChamberReport r;
  r.token = "tok123";
  r.id = "chip42";
  r.beer = beer;
  r.fridge = fridge;
  r.mode = mode;
  r.target = target;
  r.rssi = -61;
  return r;
}

static std::string text(const JsonDocument& doc) {
  std::string s;
  serializeJson(doc, s);
  return s;
}

static bool parse(const char* body, ChamberReply& reply) { return parseChamberReply(body, reply); }

int main() {
  std::printf("buildChamberPayload\n");
  {
    JsonDocument doc;
    bool ok = buildChamberPayload(makeReport(12.5f, 10.25f, 'b', 12.0f), doc);
    report("both temperatures, mode b: built", ok);
    report("token is sent, id is not", doc["token"] == "tok123" && doc["id"].isNull());
    report("beer and fridge temperature", doc["beer_temperature"].as<float>() == 12.5f && doc["fridge_temperature"].as<float>() == 10.25f);
    report("temp_units is C (the controller works in Celsius)", doc["temp_units"] == "C");
    report("current_mode B, current_target 12, rssi -61", doc["current_mode"] == "B" && doc["current_target"].as<float>() == 12.0f && doc["rssi"].as<int>() == -61);
  }
  {
    JsonDocument doc;
    ChamberReport r = makeReport(12.5f, 10.0f, 'b', 12.0f);
    r.token = "";
    buildChamberPayload(r, doc);
    report("no token: the chip id is sent instead", doc["id"] == "chip42" && doc["token"].isNull());
  }
  {
    JsonDocument doc;
    bool ok = buildChamberPayload(makeReport(NAN, 10.0f, 'f', 10.0f), doc);
    report("fridge only (beer sensor silent): built without beer_temperature", ok && doc["beer_temperature"].isNull() && doc["fridge_temperature"].as<float>() == 10.0f);
  }
  {
    JsonDocument doc;
    bool ok = buildChamberPayload(makeReport(12.0f, NAN, 'b', 12.0f), doc);
    report("beer only: built without fridge_temperature", ok && doc["fridge_temperature"].isNull() && doc["beer_temperature"].as<float>() == 12.0f);
  }
  {
    JsonDocument doc;
    bool ok = buildChamberPayload(makeReport(NAN, NAN, 'b', 12.0f), doc);
    report("no temperature at all: not built (the server would answer 422)", !ok);
  }
  {
    const char modes[] = {'b', 'f', 'o'};
    const char* want[] = {"B", "F", "O"};
    bool all = true;
    for (int i = 0; i < 3; i++) {
      JsonDocument doc;
      buildChamberPayload(makeReport(12.0f, 10.0f, modes[i], 12.0f), doc);
      all = all && doc["current_mode"] == want[i];
    }
    report("current_mode: b -> B, f -> F, o -> O", all);
    JsonDocument doc;
    buildChamberPayload(makeReport(12.0f, 10.0f, 't', 12.0f), doc);
    report("test mode t: current_mode is left out", doc["current_mode"].isNull());
  }
  {
    JsonDocument doc;
    buildChamberPayload(makeReport(12.0f, 10.0f, 'o', NAN), doc);
    report("no target: current_target is left out", doc["current_target"].isNull());
  }
  {
    JsonDocument doc;
    buildChamberPayload(makeReport(19.3f, NAN, 'b', 19.3f), doc);
    std::string s = text(doc);
    report("19.3 is written as 19.3, not 19.299999...", s.find("\"beer_temperature\":19.3,") != std::string::npos || s.find("\"beer_temperature\":19.3}") != std::string::npos);
  }

  std::printf("parseChamberReply\n");
  ChamberReply rep;
  report("B with a target", parse("{\"mode\":\"B\",\"target_temperature\":12.5,\"temp_units\":\"C\"}", rep) && rep.mode == 'B' && rep.target == 12.5f);
  report("F with a target", parse("{\"mode\":\"F\",\"target_temperature\":8,\"temp_units\":\"C\"}", rep) && rep.mode == 'F' && rep.target == 8.0f);
  report("R with a null target (the reply of a device without a batch)", parse("{\"target_temperature\":null,\"temp_units\":\"C\",\"mode\":\"R\"}", rep) && rep.mode == 'R' && std::isnan(rep.target));
  report("lower case mode is accepted", parse("{\"mode\":\"b\",\"target_temperature\":10}", rep) && rep.mode == 'B');
  report("target sent as a string: reply understood, target NAN", parse("{\"mode\":\"B\",\"target_temperature\":\"12\"}", rep) && std::isnan(rep.target));
  report("mode O: rejected (the server never answers with it)", !parse("{\"mode\":\"O\",\"target_temperature\":10}", rep));
  report("mode with two characters: rejected", !parse("{\"mode\":\"BF\",\"target_temperature\":10}", rep));
  report("no mode: rejected", !parse("{\"target_temperature\":10}", rep));
  report("mode is not a string: rejected", !parse("{\"mode\":1,\"target_temperature\":10}", rep));
  report("{} (what the framework returns for a failed request): rejected", !parse("{}", rep));
  report("not JSON: rejected", !parse("<html>502 Bad Gateway</html>", rep));
  report("empty body and null: rejected", !parse("", rep) && !parse(nullptr, rep));

  std::printf("decideChamberAction (limits 1 ... 30 C)\n");
  auto decide = [](char mode, float target, bool remote, char curMode, float curTarget) {
    ChamberReply r = {mode, target};
    return decideChamberAction(r, remote, curMode, curTarget, 1.0f, 30.0f);
  };
  {
    ChamberDecision d = decide('R', NAN, false, 'b', 12.0f);
    report("R, not in remote control: nothing (steady state, no config write)", d.action == ChamberAction::none);
    d = decide('R', NAN, true, 'b', 12.0f);
    report("R while in remote control: restore", d.action == ChamberAction::restore);
    d = decide('B', 12.0f, false, 'o', 10.0f);
    report("B, not in remote control: take control, mode b, target 12", d.action == ChamberAction::applyRemote && d.mode == 'b' && d.target == 12.0f);
    d = decide('B', 12.0f, false, 'b', 12.0f);
    report("B equal to the local setting, not remote: still takes control (locks the buttons)", d.action == ChamberAction::applyRemote);
    d = decide('F', 8.0f, true, 'f', 8.0f);
    report("F equal to the device state, in remote control: nothing", d.action == ChamberAction::none);
    d = decide('B', 12.04f, true, 'b', 12.0f);
    report("B 0.04 C from the device state: nothing", d.action == ChamberAction::none);
    d = decide('B', 12.1f, true, 'b', 12.0f);
    report("B 0.1 C from the device state: applied", d.action == ChamberAction::applyRemote && d.target == 12.1f);
    d = decide('B', 12.0f, true, 'f', 12.0f);
    report("B while the device runs f at the same target: applied", d.action == ChamberAction::applyRemote && d.mode == 'b');
    d = decide('F', 8.0f, true, 'b', NAN);
    report("F, device state has no target: applied", d.action == ChamberAction::applyRemote && d.mode == 'f');
    d = decide('B', NAN, false, 'b', 12.0f);
    report("B without a target: ignored", d.action == ChamberAction::none);
    d = decide('B', 0.5f, false, 'b', 12.0f);
    report("target below the limit (0.5 < 1): ignored", d.action == ChamberAction::none);
    d = decide('F', 31.0f, false, 'b', 12.0f);
    report("target above the limit (31 > 30): ignored", d.action == ChamberAction::none);
    d = decide('B', 1.0f, false, 'b', 12.0f);
    ChamberDecision e = decide('B', 30.0f, false, 'b', 12.0f);
    report("targets on the limits (1 and 30): applied", d.action == ChamberAction::applyRemote && e.action == ChamberAction::applyRemote);
  }

  std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASSED" : "SOMETHING FAILED", failures);
  return failures == 0 ? 0 : 1;
}
