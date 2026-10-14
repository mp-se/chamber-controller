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
#ifndef SRC_CHAMBERREPORT_HPP_
#define SRC_CHAMBERREPORT_HPP_

#include <ArduinoJson.h>

// The device side of BrewGraph's POST /ingest/chamber: what the device sends,
// how it reads the reply and what it does with it. No HTTP and no globals in
// here, so it runs in the host tests (tools/controller-sim).

struct ChamberReport {
  const char* token;  // device token, "" if there is none
  const char* id;     // chip id, sent instead of the token
  float beer;         // Celsius, NAN = no reading
  float fridge;       // Celsius, NAN = no reading
  char mode;          // controller mode: b, f, o or t
  float target;       // Celsius, NAN = none
  int rssi;
};

struct ChamberReply {
  char mode;     // 'B', 'F' or 'R' (restore, the reply of a device without a
                 // batch)
  float target;  // Celsius, NAN if the server sent none
};

enum class ChamberAction { none, applyRemote, restore };

struct ChamberDecision {
  ChamberAction action;
  char mode;     // b or f, for applyRemote
  float target;  // Celsius, for applyRemote
};

// Fills doc with the request body. Returns false if there is nothing to
// report: the server needs at least one temperature.
bool buildChamberPayload(const ChamberReport& report, JsonDocument& doc);

// Reads the reply body. Returns false if it is not a reply the device
// understands (not JSON, `{}` from a failed request, unknown mode).
bool parseChamberReply(const char* body, ChamberReply& reply);

// What to do with a reply. The common case is `R` from a device that has no
// batch: nothing at all, no config write. A target outside the controller
// limits, or none, is ignored.
ChamberDecision decideChamberAction(const ChamberReply& reply,
                                    bool remoteActive, char currentMode,
                                    float currentTarget, float minTarget,
                                    float maxTarget);

#endif  // SRC_CHAMBERREPORT_HPP_

// EOF
