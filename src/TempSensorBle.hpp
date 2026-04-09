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
#ifndef SRC_TEMPSENSORBLE_HPP_
#define SRC_TEMPSENSORBLE_HPP_

#include <TempSensor.hpp>
#include <Ticks.hpp>
#include <log.hpp>

class DallasTemperature;
class OneWire;
typedef uint8_t DeviceAddress[8];

class BleTempSensor : public BasicTempSensor {
 public:
  explicit BleTempSensor(String name) {
    Log.verbose(F("TEMP: Creating BleTempSensor %s." CR), name.c_str());
    _name = name;
  }

  ~BleTempSensor();

  bool isConnected() const;
  bool init();
  temperature read();

 private:
  String _name;
};

#endif  // SRC_TEMPSENSORBLE_HPP_

// EOF
