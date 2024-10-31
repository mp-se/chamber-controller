/*
 * Copyright 2012-2013 BrewPi/Elco Jacobs.
 * Copyright 2013 Matthew McGowan.
 *
 * This file is part of BrewPi.
 *
 * BrewPi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BrewPi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BrewPi.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SRC_CONFIG_HPP_
#define SRC_CONFIG_HPP_

#include <Arduino.h>

namespace Config {
namespace Pins {
constexpr auto heatingPin = 25;
constexpr auto coolingPin = 26;
constexpr auto oneWirePin = 13;
};  // namespace Pins

namespace TempFormat {
// Decimal places for a temperature measurement
constexpr auto tempDecimals = 1;
// Decimal places for a fixed temperature
constexpr auto fixedPointDecimals = 3;
// Decimal places for a temperature difference
constexpr auto tempDiffDecimals = 3;
// Length to use for conversion buffers
constexpr auto bufferLen = 12;
// Maximum length of a temp string
constexpr auto maxLength = 9;
};  // namespace TempFormat

namespace DeviceConfigManager {
static const int8_t MAX_DEVICES = 20;
};  // namespace DeviceConfigManager

constexpr bool forceDeviceDefaults = true;
};  // namespace Config

#define LOG_ID_TYPE \
  uint8_t  // TODO: Check what this is for, copied from Logger.h

#endif  // SRC_CONFIG_HPP_

// EOF
