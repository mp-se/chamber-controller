/*
 * Chamber Controller
 * Copyright (c) 2024-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#if defined(ENABLE_BLE) && defined(ENABLE_BLE_SENSOR)

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
    MeasurementEntry* entry = myMeasurementList.getMeasurementEntry(i);

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
    MeasurementEntry* entry = myMeasurementList.getMeasurementEntry(i);

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

#endif  // ENABLE_BLE && ENABLE_BLE_SENSOR

// EOF
