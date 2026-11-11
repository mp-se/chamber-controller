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
#ifndef SRC_CONTROLVARIABLES_HPP_
#define SRC_CONTROLVARIABLES_HPP_

#include <ArduinoJson.h>

#include <Config.hpp>
#include <TemperatureFormats.hpp>

class ControlVariables {
 public:
  ControlVariables() {}

  temperature beerDiff = 0;
  temperature diffIntegral = 0;
  temperature beerSlope = 0;
  temperature p = 0;
  temperature i = 0;
  temperature d = 0;
  temperature estimatedPeak = 0;
  temperature negPeakEstimate = 0;  // last estimate
  temperature posPeakEstimate = 0;
  temperature negPeak = 0;  // last detected peak
  temperature posPeak = 0;

  void toJsonReadable(JsonObject& doc) const {
    doc["beer_diff"] = roundTo(beerDiff, Config::TempFormat::tempDiffDecimals);
    doc["diff_integral"] =
        roundTo(diffIntegral, Config::TempFormat::tempDiffDecimals);
    doc["beer_slope"] = roundTo(beerSlope, Config::TempFormat::tempDiffDecimals);

    doc["p"] = roundTo(p, Config::TempFormat::fixedPointDecimals);
    doc["i"] = roundTo(i, Config::TempFormat::fixedPointDecimals);
    doc["d"] = roundTo(d, Config::TempFormat::fixedPointDecimals);

    doc["est_peak"] = roundTo(estimatedPeak, Config::TempFormat::tempDecimals);
    doc["neg_peak_est"] =
        roundTo(negPeakEstimate, Config::TempFormat::tempDecimals);
    doc["pos_peak_est"] =
        roundTo(posPeakEstimate, Config::TempFormat::tempDecimals);
    doc["neg_peak"] = roundTo(negPeak, Config::TempFormat::tempDecimals);
    doc["pos_peak"] = roundTo(posPeak, Config::TempFormat::tempDecimals);
  }
};

#endif  // SRC_CONTROLVARIABLES_HPP_

// EOF
