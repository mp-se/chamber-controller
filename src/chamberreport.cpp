/*
 * Chamber Controller
 * Copyright (c) 2024-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include <chamberreport.hpp>
#include <cctype>
#include <cmath>

bool buildChamberPayload(const ChamberReport& report, JsonDocument& doc) {
  if (std::isnan(report.beer) && std::isnan(report.fridge)) return false;

  if (report.token && report.token[0] != '\0') {
    doc["token"] = report.token;
  } else {
    doc["id"] = report.id;
  }

  if (!std::isnan(report.beer)) doc["beer_temperature"] = report.beer;
  if (!std::isnan(report.fridge)) doc["fridge_temperature"] = report.fridge;

  // The controller works in Celsius
  doc["temp_units"] = "C";

  switch (report.mode) {
    case 'b':
      doc["current_mode"] = "B";
      break;
    case 'f':
      doc["current_mode"] = "F";
      break;
    case 'o':
      doc["current_mode"] = "O";
      break;
    default:  // test mode has no counterpart on the server
      break;
  }

  if (!std::isnan(report.target)) doc["current_target"] = report.target;
  doc["rssi"] = report.rssi;
  return true;
}

bool parseChamberReply(const char* body, ChamberReply& reply) {
  if (!body) return false;

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) return false;

  const char* mode = doc["mode"] | "";
  if (mode[0] == '\0' || mode[1] != '\0') return false;

  char m = static_cast<char>(toupper(static_cast<unsigned char>(mode[0])));
  if (m != 'B' && m != 'F' && m != 'R') return false;

  reply.mode = m;
  reply.target = doc["target_temperature"].is<float>()
                     ? doc["target_temperature"].as<float>()
                     : NAN;
  return true;
}

ChamberDecision decideChamberAction(const ChamberReply& reply,
                                    bool remoteActive, char currentMode,
                                    float currentTarget, float minTarget,
                                    float maxTarget) {
  ChamberDecision none = {ChamberAction::none, 'o', NAN};

  if (reply.mode == 'R') {
    // Nothing to restore if BrewGraph is not in control
    if (!remoteActive) return none;
    return {ChamberAction::restore, 'o', NAN};
  }

  if (std::isnan(reply.target) || reply.target < minTarget ||
      reply.target > maxTarget) {
    return none;
  }

  char mode = reply.mode == 'B' ? 'b' : 'f';

  // Already there
  if (remoteActive && currentMode == mode &&
      std::fabs(currentTarget - reply.target) < 0.05f) {
    return none;
  }

  return {ChamberAction::applyRemote, mode, reply.target};
}

// EOF
