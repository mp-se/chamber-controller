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
#ifndef SRC_TEMPSENSOR_HPP_
#define SRC_TEMPSENSOR_HPP_

#include <stdlib.h>

#include <FilterCascaded.hpp>
#include <TempSensorBasic.hpp>

typedef CascadedFilter TempSensorFilter;
// typedef FixedFilter TempSensorFilter;

enum TempSensorType { TEMP_SENSOR_TYPE_FRIDGE = 1, TEMP_SENSOR_TYPE_BEER };

/**
 * \brief General interface for all temperature sensor devices.
 *
 * Wraps up a BasicTempSensor and processes the values through several filters.
 * The filters are either CascadedFilter or FixedFilter implementations,
 * depending on the compile time value of the TEMP_SENSOR_CASCADED_FILTER
 * macro.
 */
class TempSensor {
 public:
  explicit TempSensor(TempSensorType sensorType,
                      BasicTempSensor* sensor = NULL) {
    updateCounter = 255;  // first update for slope filter after (255-4s)
    setSensor(sensor);
  }

  void setSensor(BasicTempSensor* sensor) {
    _sensor = sensor;
    failedReadCount = -1;
  }

  bool hasSlowFilter() { return true; }
  bool hasFastFilter() { return true; }
  bool hasSlopeFilter() { return true; }
  void init();
  bool isConnected() {
    return _sensor == NULL ? false : _sensor->isConnected();
  }
  void update();
  temperature readFastFiltered();
  temperature readSlowFiltered();
  temperature readSlope();
  temperature detectPosPeak();
  temperature detectNegPeak();
  void setFastFilterCoefficients(uint8_t b);
  void setSlowFilterCoefficients(uint8_t b);
  void setSlopeFilterCoefficients(uint8_t b);
  BasicTempSensor& sensor();

 private:
  BasicTempSensor* _sensor = NULL;
  TempSensorFilter fastFilter;
  TempSensorFilter slowFilter;
  TempSensorFilter slopeFilter;
  unsigned char updateCounter;
  temperature_precise prevOutputForSlope;

  // An indication of how stale the data is in the filters.
  // Each time a read fails, this value is incremented.  It's used to reset
  // the filters after a large enough disconnect delay, and on the first
  // init.
  // `-1` for uninitialized, `>=0` afterwards.
  int8_t failedReadCount;

  friend class ChamberManager;
  friend class Chamber;
  friend class DeviceManager;
};

#endif  // SRC_TEMPSENSOR_HPP_

// EOF
