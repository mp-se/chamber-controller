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

#include <MinTimes.hpp>
#include <jsonfs.hpp>
#include <log.hpp>

constexpr auto FILENAME_CUSTOM_MIN_TIMES = "/customMinTimes.json";

// JSON Keys
constexpr auto KEY_SETTINGS_CHOICE = "settings_choice";
constexpr auto KEY_MIN_COOL_OFF_TIME = "min_cool_off_time";
constexpr auto KEY_MIN_HEAT_OFF_TIME = "min_heat_off_time";
constexpr auto KEY_MIN_COOL_ON_TIME = "min_cool_on_time";
constexpr auto KEY_MIN_HEAT_ON_TIME = "min_heat_on_time";
constexpr auto KEY_MIN_COOL_OFF_TIME_FRIDGE_CONSTANT =
    "min_cool_off_time_fridge_constant";
constexpr auto KEY_MIN_SWITCH_TIME = "min_switch_time";
constexpr auto KEY_COOL_PEAK_DETECT_TIME = "cool_peak_detect_time";
constexpr auto KEY_HEAT_PEAK_DETECT_TIME = "heat_peak_detect_time";

MinTimes::MinTimes() {
  settingsChoice = MIN_TIMES_DEFAULT;
  setDefaults();
}

void MinTimes::setDefaults(MinTimesSettingsChoice choise) {
  // Log.info(F("BREW: Using default MinTimes, %d." CR), settingsChoice);
  settingsChoice = choise;

  if (settingsChoice == MIN_TIMES_DEFAULT ||
      settingsChoice == MIN_TIMES_CUSTOM) {  // Set some defaults for custom
    MIN_COOL_OFF_TIME = 300;
    MIN_HEAT_OFF_TIME = 300;
    MIN_COOL_ON_TIME = 180;
    MIN_HEAT_ON_TIME = 180;

    MIN_COOL_OFF_TIME_FRIDGE_CONSTANT = 600;
    MIN_SWITCH_TIME = 600;
    COOL_PEAK_DETECT_TIME = 1800;
    HEAT_PEAK_DETECT_TIME = 900;
  } else if (settingsChoice == MIN_TIMES_LOW_DELAY) {
    MIN_COOL_OFF_TIME = 60;
    MIN_HEAT_OFF_TIME = 300;
    MIN_COOL_ON_TIME = 20;
    MIN_HEAT_ON_TIME = 180;

    MIN_COOL_OFF_TIME_FRIDGE_CONSTANT = 60;
    MIN_SWITCH_TIME = 600;
    COOL_PEAK_DETECT_TIME = 1800;
    HEAT_PEAK_DETECT_TIME = 900;
  }
#if defined(DEV_TESTING)
  else if (settingsChoice == MIN_TIMES_DEVELOP) {
    MIN_COOL_OFF_TIME = 6;
    MIN_HEAT_OFF_TIME = 30;
    MIN_COOL_ON_TIME = 20;
    MIN_HEAT_ON_TIME = 18;

    MIN_COOL_OFF_TIME_FRIDGE_CONSTANT = 20;
    MIN_SWITCH_TIME = 60;
    COOL_PEAK_DETECT_TIME = 180;
    HEAT_PEAK_DETECT_TIME = 90;
  }
#endif

  // If custom then we load from disk to change any of the default above.
  if (settingsChoice == MIN_TIMES_CUSTOM) {
    load();
  }
}

bool MinTimes::save() {
  JsonDocument doc;

  JsonObject obj = doc.as<JsonObject>();
  toJsonReadable(obj);
  
  JsonFileSystemHelper file(FILENAME_CUSTOM_MIN_TIMES);
  return file.saveJson(doc);
}

bool MinTimes::load() {
  setDefaults();

  JsonDocument doc;
  JsonFileSystemHelper file(FILENAME_CUSTOM_MIN_TIMES);
  bool b = file.loadJson(doc);

  if (b) {
    if (doc[KEY_SETTINGS_CHOICE].is<int>())
      settingsChoice = doc[KEY_SETTINGS_CHOICE];

    // doc the constants from the JSON Doc
    if (doc[KEY_MIN_COOL_OFF_TIME].is<int>())
      MIN_COOL_OFF_TIME = doc[KEY_MIN_COOL_OFF_TIME];
    if (doc[KEY_MIN_HEAT_OFF_TIME].is<int>())
      MIN_HEAT_OFF_TIME = doc[KEY_MIN_HEAT_OFF_TIME];
    if (doc[KEY_MIN_COOL_ON_TIME].is<int>())
      MIN_COOL_ON_TIME = doc[KEY_MIN_COOL_ON_TIME];
    if (doc[KEY_MIN_HEAT_ON_TIME].is<int>())
      MIN_HEAT_ON_TIME = doc[KEY_MIN_HEAT_ON_TIME];

    if (doc[KEY_MIN_COOL_OFF_TIME_FRIDGE_CONSTANT].is<int>())
      MIN_COOL_OFF_TIME_FRIDGE_CONSTANT =
          doc[KEY_MIN_COOL_OFF_TIME_FRIDGE_CONSTANT];
    if (doc[KEY_MIN_SWITCH_TIME].is<int>())
      MIN_SWITCH_TIME = doc[KEY_MIN_SWITCH_TIME];
    if (doc[KEY_COOL_PEAK_DETECT_TIME].is<int>())
      COOL_PEAK_DETECT_TIME = doc[KEY_COOL_PEAK_DETECT_TIME];
    if (doc[KEY_HEAT_PEAK_DETECT_TIME].is<int>())
      HEAT_PEAK_DETECT_TIME = doc[KEY_HEAT_PEAK_DETECT_TIME];
  }
  return b;
}

void MinTimes::toJsonReadable(JsonObject& doc) const {
 doc[KEY_SETTINGS_CHOICE] = settingsChoice;
  doc[KEY_MIN_COOL_OFF_TIME] = MIN_COOL_OFF_TIME;
  doc[KEY_MIN_HEAT_OFF_TIME] = MIN_HEAT_OFF_TIME;
  doc[KEY_MIN_COOL_ON_TIME] = MIN_COOL_ON_TIME;
  doc[KEY_MIN_HEAT_ON_TIME] = MIN_HEAT_ON_TIME;
  doc[KEY_MIN_COOL_OFF_TIME_FRIDGE_CONSTANT] =
      MIN_COOL_OFF_TIME_FRIDGE_CONSTANT;
  doc[KEY_MIN_SWITCH_TIME] = MIN_SWITCH_TIME;
  doc[KEY_COOL_PEAK_DETECT_TIME] = COOL_PEAK_DETECT_TIME;
  doc[KEY_HEAT_PEAK_DETECT_TIME] = HEAT_PEAK_DETECT_TIME;
}

// EOF
