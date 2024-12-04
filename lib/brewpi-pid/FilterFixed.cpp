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
 *
 * You should have received a copy of the GNU General Public License
 * along with BrewPi.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <limits.h>
#include <stdlib.h>

#include <FilterFixed.hpp>
#include <TemperatureFormats.hpp>
#include <log.hpp>

temperature FixedFilter::add(temperature val) {
  Log.verbose(F("BREW: Adding %d to FixedFilter." CR), val);
  temperature_precise returnVal = addDoublePrecision(tempRegularToPrecise(val));
  return tempPreciseToRegular(returnVal);
}

temperature_precise FixedFilter::addDoublePrecision(temperature_precise val) {
  Log.verbose(F("BREW: Adding %d (double) to FixedFilter." CR), val);

  _xv[2] = _xv[1];
  _xv[1] = _xv[0];
  _xv[0] = val;

  _yv[2] = _yv[1];
  _yv[1] = _yv[0];

  // Implementation that prevents overflow as much as possible by order of
  // operations:
  _yv[0] = ((_yv[1] - _yv[2]) + _yv[1])         // expected value + 1*
           - (_yv[1] >> _b) + (_yv[2] >> _b) +  // expected value +0*
           +(_xv[0] >> _a) + (_xv[1] >> (_a - 1)) +
           (_xv[2] >> _a)           // expected value +(1>>(a-2))
           - (_yv[2] >> (_a - 2));  // expected value -(1>>(a-2))

  return _yv[0];
}

void FixedFilter::init(temperature val) {
  Log.verbose(F("BREW: Initialize FixedFilter using %d." CR), val);

  _xv[0] = val;
  _xv[0] = tempRegularToPrecise(
      _xv[0]);  // 16 extra bits are used in the filter for the fraction part

  _xv[1] = _xv[0];
  _xv[2] = _xv[0];

  _yv[0] = _xv[0];
  _yv[1] = _xv[0];
  _yv[2] = _xv[0];
}

temperature FixedFilter::detectPosPeak() {
  if (_yv[0] < _yv[1] && _yv[1] >= _yv[2]) {
    return tempPreciseToRegular(_yv[1]);
  } else {
    Log.warning(
        F("BREW: FixedFilter invalid temperature in detecting +Peak." CR));
    return INVALID_TEMP;
  }
}

temperature FixedFilter::detectNegPeak() {
  if (_yv[0] > _yv[1] && _yv[1] <= _yv[2]) {
    return tempPreciseToRegular(_yv[1]);
  } else {
    Log.warning(
        F("BREW: FixedFilter invalid temperature in detecting -Peak." CR));
    return INVALID_TEMP;
  }
}

// EOF
