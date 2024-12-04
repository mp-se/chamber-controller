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

#include <ControlSettings.hpp>
#include <TempControl.hpp>
#include <jsonfs.hpp>
#include <log.hpp>

constexpr auto FILENAME_CONTROL_SETTINGS = "/controlSettings.json";

// JSON Keys
constexpr auto KEY_BEER_SETTING = "beer_setting";
constexpr auto KEY_FRIDGE_SETTING = "fridge_setting";
constexpr auto KEY_HEAT_EST = "heat_estimator";
constexpr auto KEY_COOL_EST = "cool_estimator";
constexpr auto KEY_MODE = "mode";

ControlSettings::ControlSettings() { setDefaults(); }

void ControlSettings::setDefaults() {
  Log.verbose(F("BREW: Using default ControlSettings." CR));

  beerSetting = intToTemp(20);
  fridgeSetting = intToTemp(20);
  heatEstimator = intToTempDiff(2) / 10;  // 0.2
  coolEstimator = intToTempDiff(5);
  mode = ControllerMode::off;  // We do NOT do call set_mode here - that is
                               // handled in TempControl::loadDefaultSettings()
}

// bool ControlSettings::save() {
//   JsonDocument doc;

//   doc[KEY_BEER_SETTING] = beerSetting;
//   doc[KEY_FRIDGE_SETTING] = fridgeSetting;
//   doc[KEY_HEAT_EST] = heatEstimator;
//   doc[KEY_COOL_EST] = coolEstimator;
//   doc[KEY_MODE] = String(mode);

//   JsonFileSystemHelper file(FILENAME_CONTROL_SETTINGS);
//   return file.saveJson(doc);
// }

// bool ControlSettings::load() {
//   setDefaults();

//   JsonDocument doc;
//   JsonFileSystemHelper file(FILENAME_CONTROL_SETTINGS);
//   bool b = file.loadJson(doc);

//   if (b) {
//     if (doc[KEY_BEER_SETTING].is<int>()) beerSetting = doc[KEY_BEER_SETTING];
//     if (doc[KEY_FRIDGE_SETTING].is<int>()) fridgeSetting = doc[KEY_FRIDGE_SETTING];
//     if (doc[KEY_HEAT_EST].is<int>()) heatEstimator = doc[KEY_HEAT_EST];
//     if (doc[KEY_COOL_EST].is<int>()) coolEstimator = doc[KEY_COOL_EST];
//     if (doc[KEY_MODE].is<String>()) mode = doc[KEY_MODE].as<String>().charAt(0);
//   }
//   return b;
// }

void ControlSettings::toJsonReadable(JsonObject& doc) const {
  doc[KEY_MODE] = String(mode);
  doc[KEY_BEER_SETTING] = tempToDouble(beerSetting, Config::TempFormat::tempDecimals);
  doc[KEY_FRIDGE_SETTING] =
      tempToDouble(fridgeSetting, Config::TempFormat::tempDecimals);
  doc[KEY_HEAT_EST] =
      fixedPointToDouble(heatEstimator, Config::TempFormat::fixedPointDecimals);
  doc[KEY_COOL_EST] =
      fixedPointToDouble(coolEstimator, Config::TempFormat::fixedPointDecimals);
}

// EOF
