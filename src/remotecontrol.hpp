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
#ifndef SRC_REMOTECONTROL_HPP_
#define SRC_REMOTECONTROL_HPP_

#include <Arduino.h>

#include <chamberreport.hpp>

// BrewGraph controls the controller mode through replies to its periodic poll.

struct RemoteResult {
  bool valid = false;    // false: not a valid request (the web handler sends 400)
  bool success = false;  // the controller accepted the mode
  String message;
};

// nullptr if the device can run the mode, otherwise why it can not
const char* remoteModeBlockedReason(char mode);

// mode is b, f, o or r (restore the settings from before BrewGraph control).
// temp is NAN when there is none. bleSensorId is reserved for legacy state and
// must be empty for BrewGraph.
RemoteResult applyRemoteCommand(char mode, float temp,
                                const String& bleSensorId);

// Applies what BrewGraph answered (see decideChamberAction)
void applyChamberReply(const ChamberReply& reply);

#endif  // SRC_REMOTECONTROL_HPP_

// EOF
