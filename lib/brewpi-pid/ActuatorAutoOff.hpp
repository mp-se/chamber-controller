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
#ifndef SRC_ACTUATORAUTOOFF_HPP_
#define SRC_ACTUATORAUTOOFF_HPP_

#include <Actuator.hpp>
#include <Ticks.hpp>
#include <log.hpp>

class AutoOffActuator : public Actuator {
 public:
  AutoOffActuator(uint16_t timeout, Actuator* target) {
    Log.verbose(F("BREW: Creating AutoOffActuator timeout=%ds target=%p." CR),
                timeout, target);
    _timeout = timeout;
    _target = target;
    _active = false;
  }

  void setActive(bool active) {
    Log.verbose(F("BREW: AutoOffActuator target=%p set active=%s." CR), _target,
                active ? "true" : "false");
    _active = active;
    _target->setActive(_active);
    if (_active) _lastActiveTime = ticks.seconds();
  }

  bool isActive() const { return _active; }

  void update() {
    Log.verbose(F("BREW: Updating AutoOffActuator target=%p." CR), _target);
    if (ticks.timeSince(_lastActiveTime) >= _timeout) setActive(false);
  }

 private:
  uint16_t _lastActiveTime;
  uint16_t _timeout;
  Actuator* _target;
  bool _active;
};

#endif  // SRC_ACTUATORAUTOOFF_HPP_

// EOF
