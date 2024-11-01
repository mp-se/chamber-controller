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
#ifndef SRC_MINTIMES_HPP_
#define SRC_MINTIMES_HPP_

#include <Arduino.h>
#include <ArduinoJson.h>

enum MinTimesSettingsChoice {
  MIN_TIMES_DEFAULT = 0,
  MIN_TIMES_LOW_DELAY = 1,
  MIN_TIMES_CUSTOM = 2,
#if defined(DEV_TESTING)
  MIN_TIMES_DEVELOP = 3
#endif
};

class MinTimes {
 private:
 public:
  MinTimes();

  MinTimesSettingsChoice settingsChoice;

  uint16_t MIN_COOL_OFF_TIME;
  uint16_t MIN_HEAT_OFF_TIME;
  uint16_t MIN_COOL_ON_TIME;
  uint16_t MIN_HEAT_ON_TIME;
  uint16_t MIN_COOL_OFF_TIME_FRIDGE_CONSTANT;
  uint16_t MIN_SWITCH_TIME;
  uint16_t COOL_PEAK_DETECT_TIME;
  uint16_t HEAT_PEAK_DETECT_TIME;

  void setDefaults(MinTimesSettingsChoice choise = MIN_TIMES_DEFAULT);
  // bool save();
  // bool load();
  void toJsonReadable(JsonObject& doc) const;
};

#endif  // SRC_MINTIMES_HPP_

// EOF
