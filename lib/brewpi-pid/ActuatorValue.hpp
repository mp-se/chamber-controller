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
#ifndef SRC_ACTUATORVALUE_HPP_
#define SRC_ACTUATORVALUE_HPP_

#include <Actuator.hpp>
#include <log.hpp>

class ValueActuator : public Actuator {
 public:
  ValueActuator() {
    Log.verbose(F("BREW: Creating ValueActuator state=false." CR));
    _state = false;
  }
  explicit ValueActuator(bool initial) {
    Log.verbose(F("BREW: Creating ValueActuator state=&s." CR),
                initial ? "true" : "false");
    _state = initial;
  }

  virtual void setActive(bool active) {
    Log.info(F("BREW: ValueActuator set active=%s." CR),
             active ? "true" : "false");
    _state = active;
  }
  virtual bool isActive() { return _state; }

 private:
  bool _state;
};

#endif  // SRC_ACTUATORVALUE_HPP_

// EOF
