/*
 * Copyright 2013 Matthew McGowan
 * Copyright 2013 BrewPi/Elco Jacobs.
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
#ifndef SRC_ACTUATORDIGITALPIN_HPP_
#define SRC_ACTUATORDIGITALPIN_HPP_

#include <Actuator.hpp>
#include <log.hpp>

class DigitalPinActuator : public Actuator {
 private:
  bool _invert;
  uint8_t _pin;
  bool _active;

 public:
  DigitalPinActuator(uint8_t pin, bool invert) {
    Log.verbose(F("BREW: Creating DigitalPinActuator pin=%d invert=%s." CR),
                pin, invert ? "true" : "false");
    _invert = invert;
    _pin = pin;
    setActive(false);
    pinMode(pin, OUTPUT);
  }

  inline virtual void setActive(bool active_setting) {
    Log.verbose(F("BREW: Set DigitalPinActuator pin=%d to valule=%s." CR), _pin,
                active_setting ? "true" : "false");
    _active = active_setting;
    digitalWrite(_pin, active_setting ^ _invert ? HIGH : LOW);
  }

  bool isActive() const { return _active; }
};

#endif  // SRC_ACTUATORDIGITALPIN_HPP_

// EOF
