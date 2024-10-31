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
#ifndef SRC_FILTERCASCADED_HPP_
#define SRC_FILTERCASCADED_HPP_

#include <FilterFixed.hpp>
#include <TemperatureFormats.hpp>

constexpr auto numFilterSections = 3;

class CascadedFilter {
 public:
  /**
   * Three filter sections gives excellent filtering, without adding too
   * much delay.  For 3 sections the stop band attenuation is 3x the single
   * section attenuation in dB.  The delay is also tripled.
   */
  FixedFilter sections[numFilterSections];

 public:
  CascadedFilter();
  ~CascadedFilter() {}
  void init(temperature val);
  void setCoefficients(uint8_t bValue);
  temperature add(temperature val);
  temperature_precise addDoublePrecision(temperature_precise val);
  temperature readInput();

  temperature readOutput() {
    return sections[numFilterSections - 1].readOutput();
  }
  temperature_precise readOutputDoublePrecision();
  temperature_precise readPrevOutputDoublePrecision();

  temperature detectPosPeak() {
    return sections[numFilterSections - 1].detectPosPeak();
  }

  temperature detectNegPeak() {
    return sections[numFilterSections - 1].detectNegPeak();
  }
};

#endif  // SRC_FILTERCASCADED_HPP_

// EOF
