/*
 * Copyright 2013 BrewPi/Elco Jacobs.
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

#include <Ticks.hpp>

HardwareTicks ticks = HardwareTicks();
HardwareDelay wait = HardwareDelay();

ticks_seconds_t HardwareTicks::secondsSince(ticks_millis_t timeStamp) {
  // Unsigned subtraction gives the right answer across the millis() rollover
  ticks_millis_t elapsed = ::millis() - timeStamp;
  return elapsed / 1000;
}

ticks_seconds_t HardwareTicks::seconds() { return ::millis() / 1000; }

void HardwareDelay::millis(uint16_t millis) { ::delay(millis); }
void HardwareDelay::seconds(uint16_t seconds) { ::delay(seconds * 1000UL); }

// EOF
