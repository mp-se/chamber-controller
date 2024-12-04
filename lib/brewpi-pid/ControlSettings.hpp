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
#ifndef SRC_CONTROLSETTINGS_HPP_
#define SRC_CONTROLSETTINGS_HPP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <TemperatureFormats.hpp>

struct ControlSettings {
 public:
  ControlSettings();

  temperature beerSetting;
  temperature fridgeSetting;
  temperature
      heatEstimator;  // updated automatically by self learning algorithm
  temperature
      coolEstimator;  // updated automatically by self learning algorithm
  char mode;

  void setDefaults();
  bool save();
  bool load();
  void toJsonReadable(JsonObject& doc) const;
  void fromJsonReadable(JsonObject& doc);
};

#endif  // SRC_CONTROLSETTINGS_HPP_

// EOF
