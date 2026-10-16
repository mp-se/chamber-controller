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
#include <espframework.hpp>
#include <log.hpp>
#include <pidconfig.hpp>
#include <pidpush.hpp>

PidPush::PidPush(PidConfig* config) : BasePush(config) { _pidConfig = config; }

bool PidPush::pollChamber(const ChamberReport& report, ChamberReply& reply) {
  JsonDocument doc;
  if (!buildChamberPayload(report, doc)) {
    Log.info(F("PUSH: BrewGraph, no temperature to report." CR));
    return false;
  }

  // The body holds the device token, it is not logged
  String body;
  serializeJson(doc, body);

  String response =
      sendHttpPost(body, _pidConfig->getBrewgraphUrl(),
                   "Content-Type: application/json", "");
  _brewgraphStatus.update(*this, response);

  if (!wasLastSuccessful()) {
    Log.warning(F("PUSH: BrewGraph poll failed, response code %d." CR),
                getLastResponseCode());
    return false;
  }

  if (!parseChamberReply(response.c_str(), reply)) {
    Log.warning(F("PUSH: BrewGraph reply not understood." CR));
    return false;
  }

  return true;
}

// EOF
