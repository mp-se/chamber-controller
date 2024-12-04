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
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BrewPi.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits.h>
#include <stdlib.h>

#include <FilterCascaded.hpp>
#include <FilterFixed.hpp>
#include <TemperatureFormats.hpp>
#include <log.hpp>

CascadedFilter::CascadedFilter() {
  Log.verbose(F("BREW: Creating CascadedFilter." CR));

  setCoefficients(2);
}

void CascadedFilter::setCoefficients(uint8_t bValue) {
  Log.verbose(F("BREW: Setting coefficients on CascadedFilter to %d." CR),
  bValue);

  for (uint8_t i = 0; i < numFilterSections; i++) {
    sections[i].setCoefficients(bValue);
  }
}

temperature CascadedFilter::add(temperature val) {
  Log.verbose(F("BREW: Adding temp %d to CascadedFilter." CR), val);

  temperature_precise valDoublePrecision = tempRegularToPrecise(val);
  valDoublePrecision = addDoublePrecision(valDoublePrecision);
  return tempPreciseToRegular(valDoublePrecision);
}

temperature_precise CascadedFilter::addDoublePrecision(
    temperature_precise val) {
  Log.verbose(F("BREW: Adding temp %d (double) to CascadedFilter." CR), val);

  temperature_precise input = val;

  for (uint8_t i = 0; i < numFilterSections; i++) {
    input = sections[i].addDoublePrecision(input);
  }

  return input;
}

temperature CascadedFilter::readInput() { return sections[0].readInput(); }

temperature_precise CascadedFilter::readOutputDoublePrecision() {
  return sections[numFilterSections - 1].readOutputDoublePrecision();
}

temperature_precise CascadedFilter::readPrevOutputDoublePrecision() {
  return sections[numFilterSections - 1].readPrevOutputDoublePrecision();
}

void CascadedFilter::init(temperature val) {
  Log.verbose(F("BREW: Initialize CascadedFilter with %d." CR), val);

  for (uint8_t i = 0; i < numFilterSections; i++) {
    sections[i].init(val);
  }
}

// EOF
