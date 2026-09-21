/*
 * Copyright 2012-2013 BrewPi/Elco Jacobs.
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

#include <stdio.h>

#include <Config.hpp>
#include <TempSensor.hpp>
#include <Ticks.hpp>
#include <log.hpp>

void TempSensor::init() {
  Log.verbose(F("BREW: Initializing TempSensor." CR));

  if (_sensor && _sensor->init() &&
      (failedReadCount < 0 || failedReadCount > 60)) {
    temperature temp = _sensor->read();

    if (!isnan(temp)) {
      initFilters(temp);
    } else {
      Log.warning(
          F("BREW: Initializing TempSensor failed, no temperature." CR));
    }
  }
}

void TempSensor::initFilters(temperature temp) {
  Log.verbose(F("BREW: Initializing TempSensor filters with %F." CR), temp);

  fastFilter.init(temp);
  slowFilter.init(temp);
  slopeFilter.init(0);
  prevOutputForSlope = slowFilter.readOutput();
  failedReadCount = 0;
}

void TempSensor::update() {
  Log.verbose(F("BREW: Updating TempSensor." CR));

  temperature temp = _sensor ? _sensor->read() : TEMP_SENSOR_DISCONNECTED;
  if (isnan(temp)) {
    if (failedReadCount >= 0) failedReadCount++;
    failedReadCount = min(failedReadCount, int8_t(127));  // limit
    return;
  }

  // The filters hold no data before the first valid reading (init() can find
  // no reading, for example a BLE sensor that has not been heard yet) and only
  // old data after a long disconnect. Start them from this reading in both
  // cases, then count consecutive failed reads only.
  if (failedReadCount < 0 || failedReadCount > 60) {
    initFilters(temp);
  }
  failedReadCount = 0;

  Log.verbose(F("BREW: Updating TempSensor filters with temp %F." CR), temp);

  fastFilter.add(temp);
  slowFilter.add(temp);

  // update slope filter every 3 samples.
  // averaged differences will give the slope. Use the slow filter as input
  updateCounter--;
  // initialize first read for slope filter after (255-4) seconds. This prevents
  // an influence for the startup inaccuracy.
  if (updateCounter == 4) {
    // only happens once after startup.
    prevOutputForSlope = slowFilter.readOutput();
  }
  if (updateCounter == 0) {
    temperature slowFilterOutput = slowFilter.readOutput();
    temperature diff = slowFilterOutput - prevOutputForSlope;
    slopeFilter.add(1200 * diff);  // Multiply by 1200 (1h/4s)
    prevOutputForSlope = slowFilterOutput;
    updateCounter = 3;
  }
}

String TempSensor::getSensorName() const {
  return _sensor ? _sensor->getName() : "Unknown";
}

temperature TempSensor::readFastFiltered() {
  return fastFilter.readOutput();  // return most recent unfiltered value
}

temperature TempSensor::readSlowFiltered() {
  return slowFilter.readOutput();  // return most recent unfiltered value
}

temperature TempSensor::readSlope() {
  // return slope per hour.
  return slopeFilter.readOutput();
}

temperature TempSensor::detectPosPeak() { return slowFilter.detectPosPeak(); }

temperature TempSensor::detectNegPeak() { return slowFilter.detectNegPeak(); }

void TempSensor::setFastFilterCoefficients(uint8_t b) {
  fastFilter.setCoefficients(b);
}

void TempSensor::setSlowFilterCoefficients(uint8_t b) {
  slowFilter.setCoefficients(b);
}

void TempSensor::setSlopeFilterCoefficients(uint8_t b) {
  slopeFilter.setCoefficients(b);
}

BasicTempSensor& TempSensor::sensor() { return *_sensor; }

// EOF
