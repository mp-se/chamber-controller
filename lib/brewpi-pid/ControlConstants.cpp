/*
 * Copyright 2013 BrewPi/Elco Jacobs.
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

#include <ArduinoJson.h>

#include <Config.hpp>
#include <ControlConstants.hpp>
#include <jsonfs.hpp>
#include <log.hpp>

constexpr auto FILENAME_CONTROL_CONSTANTS = "/controlConstants.json";

// JSON Keys
constexpr auto KEY_SETTINGS_CHOICE = "settings_choice";

constexpr auto KEY_TEMP_SETTING_MIN = "temp_setting_min";
constexpr auto KEY_TEMP_SETTING_MAX = "temp_setting_max";
constexpr auto KEY_KP = "kp";
constexpr auto KEY_KI = "ki";
constexpr auto KEY_KD = "kd";
constexpr auto KEY_MAX_ERROR = "i_max_error";
constexpr auto KEY_IDLE_RANGE_HIGH = "idle_range_high";
constexpr auto KEY_IDLE_RANGE_LOW = "idle_range_low";
constexpr auto KEY_HEATING_TARGET_UPPER = "heating_target_upper";
constexpr auto KEY_HEATING_TARGET_LOWER = "heating_target_lower";
constexpr auto KEY_COOLING_TARGET_UPPER = "cooling_target_upper";
constexpr auto KEY_COOLING_TARGET_LOWER = "cooling_target_lower";
constexpr auto KEY_MAX_HEAT_EST = "max_heat_time_for_estimate";
constexpr auto KEY_MAX_COOL_EST = "max_cool_time_for_estimate";
constexpr auto KEY_FRIDGE_FILTER_FAST = "fridge_fast_filter";
constexpr auto KEY_FRIDGE_FILTER_SLOW = "fridge_slow_filter";
constexpr auto KEY_FRIDGE_FILTER_SLOPE = "fridge_slope_filter";
constexpr auto KEY_BEER_FILTER_FAST = "beer_fast_filter";
constexpr auto KEY_BEER_FILTER_SLOW = "beer_slow_filter";
constexpr auto KEY_BEER_FILTER_SLOPE = "beer_slope_filter";
constexpr auto KEY_PID_MAX = "pid_max";
constexpr auto KEY_TEMP_FORMAT = "temp_format";

ControlConstants::ControlConstants() { setDefaults(); }

void ControlConstants::setDefaults() {
  Log.verbose(F("BREW: Using default ControlConstants." CR));

  tempSettingMin = intToTemp(1);   // +1 deg Celsius
  tempSettingMax = intToTemp(30);  // +30 deg Celsius

  // control defines, also in fixed point format (7 int bits, 9 frac bits), so
  // multiplied by 2^9=512
  Kp = intToTempDiff(5);              // +5
  Ki = intToTempDiff(1) / 4;          // +0.25
  Kd = intToTempDiff(-3) / 2;         // -1.5
  iMaxError = intToTempDiff(5) / 10;  // 0.5 deg

  // Stay Idle when fridge temperature is in this range
  idleRangeHigh = intToTempDiff(1);  // +1 deg Celsius
  idleRangeLow = intToTempDiff(-1);  // -1 deg Celsius

  // when peak falls between these limits, its good.
  heatingTargetUpper = intToTempDiff(3) / 10;   // +0.3 deg Celsius
  heatingTargetLower = intToTempDiff(-2) / 10;  // -0.2 deg Celsius
  coolingTargetUpper = intToTempDiff(2) / 10;   // +0.2 deg Celsius
  coolingTargetLower = intToTempDiff(-3) / 10;  // -0.3 deg Celsius

  // maximum history to take into account, in seconds
  maxHeatTimeForEstimate = 600;
  maxCoolTimeForEstimate = 1200;

  // Set filter coefficients. This is the b value. See FilterFixed.h for delay
  // times. The delay time is 3.33 * 2^b * number of cascades
  fridgeFastFilter = 1u;
  fridgeSlowFilter = 4u;
  fridgeSlopeFilter = 3u;
  beerFastFilter = 3u;
  beerSlowFilter = 4u;
  beerSlopeFilter = 4u;

  pidMax = intToTempDiff(10);  // +/- 10 deg Celsius
  tempFormat = 'C';
}

// bool ControlConstants::save() {
//   JsonDocument doc;

//   doc[KEY_TEMP_SETTING_MIN] = tempSettingMin;
//   doc[KEY_TEMP_SETTING_MAX] = tempSettingMax;
//   doc[KEY_KP] = Kp;
//   doc[KEY_KI] = Ki;
//   doc[KEY_KD] = Kd;
//   doc[KEY_MAX_ERROR] = iMaxError;
//   doc[KEY_IDLE_RANGE_HIGH] = idleRangeHigh;
//   doc[KEY_IDLE_RANGE_LOW] = idleRangeLow;
//   doc[KEY_HEATING_TARGET_UPPER] = heatingTargetUpper;
//   doc[KEY_HEATING_TARGET_LOWER] = heatingTargetLower;
//   doc[KEY_COOLING_TARGET_UPPER] = coolingTargetUpper;
//   doc[KEY_COOLING_TARGET_LOWER] = coolingTargetLower;
//   doc[KEY_MAX_HEAT_EST] = maxHeatTimeForEstimate;
//   doc[KEY_MAX_COOL_EST] = maxCoolTimeForEstimate;
//   doc[KEY_FRIDGE_FILTER_FAST] = fridgeFastFilter;
//   doc[KEY_FRIDGE_FILTER_SLOW] = fridgeSlowFilter;
//   doc[KEY_FRIDGE_FILTER_SLOPE] = fridgeSlopeFilter;
//   doc[KEY_BEER_FILTER_FAST] = beerFastFilter;
//   doc[KEY_BEER_FILTER_SLOW] = beerSlowFilter;
//   doc[KEY_BEER_FILTER_SLOPE] = beerSlopeFilter;
//   doc[KEY_PID_MAX] = pidMax;
//   doc[KEY_TEMP_FORMAT] = String(tempFormat);

//   JsonFileSystemHelper file(FILENAME_CONTROL_CONSTANTS);
//   return file.saveJson(doc);
// }

// bool ControlConstants::load() {
//   setDefaults();

//   JsonDocument doc;
//   JsonFileSystemHelper file(FILENAME_CONTROL_CONSTANTS);
//   bool b = file.loadJson(doc);

//   if (b) {
//     if (doc[KEY_TEMP_SETTING_MIN].is<int>()) tempSettingMin = doc[KEY_TEMP_SETTING_MIN];
//     if (doc[KEY_TEMP_SETTING_MAX].is<int>()) tempSettingMax = doc[KEY_TEMP_SETTING_MAX];
//     if (doc[KEY_KP].is<int>()) Kp = doc[KEY_KP];
//     if (doc[KEY_KI].is<int>()) Ki = doc[KEY_KI];
//     if (doc[KEY_KD].is<int>()) Kd = doc[KEY_KD];
//     if (doc[KEY_MAX_ERROR].is<int>()) iMaxError = doc[KEY_MAX_ERROR];
//     if (doc[KEY_IDLE_RANGE_HIGH].is<int>()) idleRangeHigh = doc[KEY_IDLE_RANGE_HIGH];
//     if (doc[KEY_IDLE_RANGE_LOW].is<int>()) idleRangeLow = doc[KEY_IDLE_RANGE_LOW];
//     if (doc[KEY_HEATING_TARGET_UPPER].is<int>())
//       heatingTargetUpper = doc[KEY_HEATING_TARGET_UPPER];
//     if (doc[KEY_HEATING_TARGET_LOWER].is<int>())
//       heatingTargetLower = doc[KEY_HEATING_TARGET_LOWER];
//     if (doc[KEY_COOLING_TARGET_UPPER].is<int>())
//       coolingTargetUpper = doc[KEY_COOLING_TARGET_UPPER];
//     if (doc[KEY_COOLING_TARGET_LOWER].is<int>())
//       coolingTargetLower = doc[KEY_COOLING_TARGET_LOWER];
//     if (doc[KEY_MAX_HEAT_EST].is<int>())
//       maxHeatTimeForEstimate = doc[KEY_MAX_HEAT_EST];
//     if (doc[KEY_MAX_COOL_EST].is<int>())
//       maxCoolTimeForEstimate = doc[KEY_MAX_COOL_EST];
//     if (doc[KEY_FRIDGE_FILTER_FAST].is<int>())
//       fridgeFastFilter = doc[KEY_FRIDGE_FILTER_FAST];
//     if (doc[KEY_FRIDGE_FILTER_SLOW].is<int>())
//       fridgeSlowFilter = doc[KEY_FRIDGE_FILTER_SLOW];
//     if (doc[KEY_FRIDGE_FILTER_SLOPE].is<int>())
//       fridgeSlopeFilter = doc[KEY_FRIDGE_FILTER_SLOPE];
//     if (doc[KEY_BEER_FILTER_FAST].is<int>())
//       beerFastFilter = doc[KEY_BEER_FILTER_FAST];
//     if (doc[KEY_BEER_FILTER_SLOW].is<int>())
//       beerSlowFilter = doc[KEY_BEER_FILTER_SLOW];
//     if (doc[KEY_BEER_FILTER_SLOPE].is<int>())
//       beerSlopeFilter = doc[KEY_BEER_FILTER_SLOPE];
//     // if (doc[KEY_LIGHT_HEATER].is<int>()) lightAsHeater =
//     // doc[KEY_LIGHT_HEATER]; if (doc[KEY_ROTARY_HALF_STEPS].is<int>())
//     //   rotaryHalfSteps = doc[KEY_ROTARY_HALF_STEPS];
//     if (doc[KEY_PID_MAX].is<int>()) pidMax = doc[KEY_PID_MAX];
//     if (doc[KEY_TEMP_FORMAT].is<int>())
//       tempFormat = doc[KEY_TEMP_FORMAT].as<String>().charAt(0);
//   }
//   return b;
// }

void ControlConstants::toJsonReadable(JsonObject& doc) const {
  doc[KEY_TEMP_FORMAT] = String(tempFormat);

  doc[KEY_TEMP_SETTING_MIN] =
      tempToDouble(tempSettingMin, Config::TempFormat::tempDecimals);
  doc[KEY_TEMP_SETTING_MAX] =
      tempToDouble(tempSettingMax, Config::TempFormat::tempDecimals);
  doc[KEY_PID_MAX] =
      tempDiffToDouble(pidMax, Config::TempFormat::tempDiffDecimals);
  doc[KEY_KP] = fixedPointToDouble(Kp, Config::TempFormat::fixedPointDecimals);
  doc[KEY_KI] = fixedPointToDouble(Ki, Config::TempFormat::fixedPointDecimals);
  doc[KEY_KD] = fixedPointToDouble(Kd, Config::TempFormat::fixedPointDecimals);


  doc[KEY_MAX_ERROR] =
      tempDiffToDouble(iMaxError, Config::TempFormat::tempDiffDecimals);
  doc[KEY_IDLE_RANGE_HIGH] =
      tempDiffToDouble(idleRangeHigh, Config::TempFormat::tempDiffDecimals);
  doc[KEY_IDLE_RANGE_LOW] =
      tempDiffToDouble(idleRangeLow, Config::TempFormat::tempDiffDecimals);
  doc[KEY_HEATING_TARGET_UPPER] = tempDiffToDouble(heatingTargetUpper,
                                        Config::TempFormat::tempDiffDecimals);
  doc[KEY_HEATING_TARGET_LOWER] = tempDiffToDouble(heatingTargetLower,
                                        Config::TempFormat::tempDiffDecimals);
  doc[KEY_COOLING_TARGET_UPPER] = tempDiffToDouble(coolingTargetUpper,
                                        Config::TempFormat::tempDiffDecimals);
  doc[KEY_COOLING_TARGET_LOWER] = tempDiffToDouble(coolingTargetLower,
                                        Config::TempFormat::tempDiffDecimals);
  doc[KEY_MAX_HEAT_EST] = maxHeatTimeForEstimate;
  doc[KEY_MAX_COOL_EST] = maxCoolTimeForEstimate;
  doc[KEY_FRIDGE_FILTER_FAST] = fridgeFastFilter;
  doc[KEY_FRIDGE_FILTER_SLOW] = fridgeSlowFilter;
  doc[KEY_FRIDGE_FILTER_SLOPE] = fridgeSlopeFilter;
  doc[KEY_BEER_FILTER_FAST] = beerFastFilter;
  doc[KEY_BEER_FILTER_SLOW] = beerSlowFilter;
  doc[KEY_BEER_FILTER_SLOPE] = beerSlopeFilter;
}

// EOF
