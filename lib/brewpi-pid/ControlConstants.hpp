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
#ifndef SRC_CONTROLCONSTANTS_HPP_
#define SRC_CONTROLCONSTANTS_HPP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <TemperatureFormats.hpp>

class ControlConstants {
 public:
  ControlConstants();

  temperature tempSettingMin;
  temperature tempSettingMax;
  temperature Kp;
  temperature Ki;
  temperature Kd;
  temperature iMaxError;
  temperature idleRangeHigh;
  temperature idleRangeLow;
  temperature heatingTargetUpper;
  temperature heatingTargetLower;
  temperature coolingTargetUpper;
  temperature coolingTargetLower;
  uint16_t maxHeatTimeForEstimate;  //!< max time for heat estimate in seconds
  uint16_t maxCoolTimeForEstimate;  //!< max time for heat estimate in seconds
  // for the filter coefficients the b value is stored. a is calculated from b.
  uint8_t fridgeFastFilter;   //!< for display, logging and on-off control
  uint8_t fridgeSlowFilter;   //!< for peak detection
  uint8_t fridgeSlopeFilter;  //!< not used in current control algorithm
  uint8_t beerFastFilter;     //!< for display and logging
  uint8_t beerSlowFilter;     //!< for on/off control algorithm
  uint8_t beerSlopeFilter;    //!< for PID calculation
  // uint8_t lightAsHeater;  //!< Use the light to heat rather than the
  // configured
  //                         //!< heater device
  // uint8_t rotaryHalfSteps;  //!< Define whether to use full or half steps for
  //                           //!< the rotary encoder
  temperature pidMax;
  char tempFormat;

  void setDefaults();
  bool save();
  bool load();
  void toJsonReadable(JsonObject& doc) const;
};

#endif  // SRC_CONTROLCONSTANTS_HPP_

// EOF
