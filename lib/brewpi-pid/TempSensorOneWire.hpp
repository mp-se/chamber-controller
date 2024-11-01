/*
 * Copyright 2012-2013 BrewPi/Elco Jacobs.
 * Copyright 2013 Matthew McGowan
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
#ifndef SRC_TEMPSENSORONEWIRE_HPP_
#define SRC_TEMPSENSORONEWIRE_HPP_

#include <TempSensor.hpp>
#include <Ticks.hpp>
#include <log.hpp>

class DallasTemperature;
class OneWire;
typedef uint8_t DeviceAddress[8];

class OneWireTempSensor : public BasicTempSensor {
 public:
  OneWireTempSensor(OneWire* bus, DeviceAddress address,
                    fixed4_4 calibrationOffset) {
    // Log.verbose(F("TEMP: Creating OneWireTempSensor with offset %d." CR),
    //             calibrationOffset);

    _oneWire = bus;
    _sensor = NULL;
    _connected = true;
    memcpy(_sensorAddress, address, sizeof(DeviceAddress));
    _calibrationOffset = calibrationOffset;
  }

  ~OneWireTempSensor();

  bool isConnected() { return _connected; }

  bool init();
  temperature read();

 private:
  constexpr static uint8_t _sensorPrecision = 4;

  void setConnected(bool connected);
  bool requestConversion();
  void waitForConversion() { wait.millis(750); }
  temperature readAndConstrainTemp();

  OneWire* _oneWire;
  DallasTemperature* _sensor;
  DeviceAddress _sensorAddress;
  fixed4_4 _calibrationOffset;
  bool _connected;
};

#endif  // SRC_TEMPSENSORONEWIRE_HPP_

// EOF
