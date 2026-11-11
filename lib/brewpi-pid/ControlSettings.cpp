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

  beerSetting = 20;
  fridgeSetting = 20;
  heatEstimator = 0.2;
  coolEstimator = 5;
  mode = ControllerMode::off;  // We do NOT do call set_mode here - that is
                               // handled in TempControl::loadDefaultSettings()
}

bool ControlSettings::save() {
#if defined(BREWPI_ENABLE_SAVE)
  JsonDocument doc;
  JsonObject obj = doc.to<JsonObject>();
  toJsonReadable(obj);

  JsonFileSystemHelper file(FILENAME_CONTROL_SETTINGS);
  return file.saveJson(doc);
#else
  return true;
#endif
}

bool ControlSettings::load() {
#if defined(BREWPI_ENABLE_SAVE)
  setDefaults();

  JsonDocument doc;
  JsonFileSystemHelper file(FILENAME_CONTROL_SETTINGS);
  bool b = file.loadJson(doc);

  if (b) {
    JsonObject obj = doc.as<JsonObject>();
    fromJsonReadable(obj);
  }
  return b;
#else
  return true;
#endif
}

void ControlSettings::toJsonReadable(JsonObject& doc) const {
  doc[KEY_MODE] = String(mode);
  doc[KEY_BEER_SETTING] = roundTo(beerSetting, Config::TempFormat::tempDecimals);
  doc[KEY_FRIDGE_SETTING] =
      roundTo(fridgeSetting, Config::TempFormat::tempDecimals);
  doc[KEY_HEAT_EST] = roundTo(heatEstimator, Config::TempFormat::fixedPointDecimals);
  doc[KEY_COOL_EST] = roundTo(coolEstimator, Config::TempFormat::fixedPointDecimals);
}

void ControlSettings::fromJsonReadable(JsonObject& doc) {
  if (doc[KEY_BEER_SETTING].is<double>()) beerSetting = doc[KEY_BEER_SETTING];
  if (doc[KEY_FRIDGE_SETTING].is<double>())
    fridgeSetting = doc[KEY_FRIDGE_SETTING];
  if (doc[KEY_HEAT_EST].is<double>()) heatEstimator = doc[KEY_HEAT_EST];
  if (doc[KEY_COOL_EST].is<double>()) coolEstimator = doc[KEY_COOL_EST];
  if (doc[KEY_MODE].is<String>()) mode = doc[KEY_MODE].as<String>().charAt(0);
}

// EOF
