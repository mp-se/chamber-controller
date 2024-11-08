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

#include <ActuatorDigitalPin.hpp>
#include <Config.hpp>
#include <DallasTempNG.hpp>
#include <NumberFormats.hpp>
#include <TempSensorOneWire.hpp>
#include <TemperatureFormats.hpp>
#include <Ticks.hpp>
#include <log.hpp>

OneWireTempSensor::~OneWireTempSensor() { delete _sensor; }

bool OneWireTempSensor::init() {
  // Log.info(F("BREW: Initializing OneWireTempSensor" CR));

  char addressString[17] = "";
  printBytes(_sensorAddress, 8, addressString);

  bool success = false;

  if (_sensor == NULL) {
    _sensor = new DallasTemperature(_oneWire);
    if (_sensor == NULL) {
      Log.error(F("BREW: Failed to create DallasTempSensor ." CR));
    }
  }

  // This quickly tests if the sensor is connected and initializes the reset
  // detection. During the main TempControl loop, we don't want to spend many
  // seconds scanning each sensor since this brings things to a halt.
  if (_sensor && initConnection(*_sensor, _sensorAddress) &&
      requestConversion()) {
    Log.verbose(F("BREW: Waiting for OneWire sensor to respond." CR));
    waitForConversion();
    temperature temp = readAndConstrainTemp();
    Log.verbose(F("BREW: Reading temp %F from sensor at %s." CR),
                tempToDouble(temp, Config::TempFormat::tempDecimals),
                addressString);
    success = temp != TEMP_SENSOR_DISCONNECTED && requestConversion();
  }
  setConnected(success);
  Log.verbose(F("BREW: OneWire sensor initialized correctly." CR));
  return success;
}

bool OneWireTempSensor::requestConversion() {
  bool ok = _sensor->requestTemperaturesByAddress(_sensorAddress);
  setConnected(ok);
  return ok;
}

void OneWireTempSensor::setConnected(bool connected) {
  if (_connected == connected) return;  // state is stays the same

  char addressString[17];
  printBytes(_sensorAddress, 8, addressString);
  _connected = connected;
  Log.info(F("BREW: TempSensor with adress %s %s." CR), addressString,
           connected ? "connected" : "disconnected");
}

temperature OneWireTempSensor::read() {
  if (!_connected) return TEMP_SENSOR_DISCONNECTED;

  temperature temp = readAndConstrainTemp();
  requestConversion();
  return temp;
}

temperature OneWireTempSensor::readAndConstrainTemp() {
  temperature temp = getTempRaw(*_sensor, _sensorAddress);

  if (temp == DEVICE_DISCONNECTED_RAW) {
    setConnected(false);
    return TEMP_SENSOR_DISCONNECTED;
  }

  const uint8_t shift = TEMP_FIXED_POINT_BITS -
                        _sensorPrecision;  // difference in precision between
                                           // DS18B20 format and temperature adt
  temp = constrainTemp(temp + _calibrationOffset + (C_OFFSET >> shift),
                       (static_cast<int>(MIN_TEMP)) >> shift,
                       (static_cast<int>(MAX_TEMP)) >> shift)
         << shift;
  return temp;
}

// EOF
