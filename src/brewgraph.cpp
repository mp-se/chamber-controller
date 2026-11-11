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
#include <brewgraph.hpp>

BrewGraphPush::BrewGraphPush(PidConfig* config) : BasePush(config) {
  _pidConfig = config;
}

bool BrewGraphPush::pollChamber(const ChamberReport& report,
                                ChamberReply& reply) {
  JsonDocument doc;
  if (!buildChamberPayload(report, doc)) {
    Log.info(F("BREW: No temperature to report." CR));
    return false;
  }

  String body;
  serializeJson(doc, body);

  Log.info(F("BREW: Request: %s" CR), body.c_str());

  String response =
      sendHttpPost(body, _pidConfig->getBrewgraphUrl(),
                   "Content-Type: application/json", "");
  _brewgraphStatus.update(*this, response);

  if (!wasLastSuccessful()) {
    Log.warning(F("BREW: Poll failed, response code %d." CR),
                getLastResponseCode());
    return false;
  }

  Log.info(F("BREW: Reply: %s" CR), response.c_str());

  if (!parseChamberReply(response.c_str(), reply)) {
    Log.warning(F("BREW: Reply not understood." CR));
    return false;
  }

  return true;
}

// EOF
