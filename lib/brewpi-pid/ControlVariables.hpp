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

  temperature beerDiff;
  long_temperature diffIntegral;  // also uses 9 fraction bits, but more integer
                                  // bits to prevent overflow
  temperature beerSlope;
  temperature p;
  temperature i;
  temperature d;
  temperature estimatedPeak;
  temperature negPeakEstimate;  // last estimate
  temperature posPeakEstimate;
  temperature negPeak;  // last detected peak
  temperature posPeak;

  void toJsonReadable(JsonObject& doc) const {
    doc["beer_diff"] =
        tempDiffToDouble(beerDiff, Config::TempFormat::tempDiffDecimals);
    doc["diff_integral"] =
        tempDiffToDouble(diffIntegral, Config::TempFormat::tempDiffDecimals);
    doc["beer_slope"] =
        tempDiffToDouble(beerSlope, Config::TempFormat::tempDiffDecimals);

    doc["p"] = fixedPointToDouble(p, Config::TempFormat::fixedPointDecimals);
    doc["i"] = fixedPointToDouble(i, Config::TempFormat::fixedPointDecimals);
    doc["d"] = fixedPointToDouble(d, Config::TempFormat::fixedPointDecimals);

    doc["est_peak"] =
        tempToDouble(estimatedPeak, Config::TempFormat::tempDecimals);
    doc["neg_peak_est"] =
        tempToDouble(negPeakEstimate, Config::TempFormat::tempDecimals);
    doc["pos_peak_est"] =
        tempToDouble(posPeakEstimate, Config::TempFormat::tempDecimals);
    doc["neg_peak"] = tempToDouble(negPeak, Config::TempFormat::tempDecimals);
    doc["pos_peak"] = tempToDouble(posPeak, Config::TempFormat::tempDecimals);
  }
};

#endif  // SRC_CONTROLVARIABLES_HPP_

// EOF
