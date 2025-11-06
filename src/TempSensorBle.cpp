/*
MIT License

Copyright (c) 2024-2025 Magnus

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <ActuatorDigitalPin.hpp>
#include <Config.hpp>
#include <DallasTempNG.hpp>
#include <NumberFormats.hpp>
#include <TempSensorBle.hpp>
#include <TemperatureFormats.hpp>
#include <Ticks.hpp>
#include <log.hpp>
#include <measurement.hpp>
#include <pidconfig.hpp>

BleTempSensor::~BleTempSensor() {}

bool BleTempSensor::init() {
  Log.info(F("BREW: Initializing BleTempSensor" CR));
  return true;
}

bool BleTempSensor::isConnected() const {
  for (int i = 0; i < myMeasurementList.size(); i++) {
    MeasurementEntry *entry = myMeasurementList.getMeasurementEntry(i);

    if (entry->getId() == _name) {
      // Consider the sensor connected if data is updated within the valid time
      // frame
      return entry->getUpdateAge() < (myConfig.getBleSensorValidTime() * 60);
    }
  }

  return false;
}

temperature BleTempSensor::read() {
  for (int i = 0; i < myMeasurementList.size(); i++) {
    MeasurementEntry *entry = myMeasurementList.getMeasurementEntry(i);

    if (entry->getId() == _name) {
      // Consider the sensor connected if data is updated within the valid time
      // frame
      if (entry->getUpdateAge() < (myConfig.getBleSensorValidTime() * 60)) {
        switch (entry->getType()) {
          case MeasurementType::Gravitymon:
            return doubleToTemp(entry->getGravityData()->getTempC());
            break;

          case MeasurementType::Tilt:
          case MeasurementType::TiltPro:
            return doubleToTemp(entry->getTiltData()->getTempC());
            break;

          case MeasurementType::Rapt:
            return doubleToTemp(entry->getRaptData()->getTempC());
            break;
        }
      }
    }
  }

  return INVALID_TEMP;
}

// EOF
