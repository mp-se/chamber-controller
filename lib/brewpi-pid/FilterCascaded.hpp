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

#include <TemperatureFormats.hpp>

/**
 * This class implements an IIR low pass filter, with the following frequency
 * response
 *
 * \f[
 * H(z) = 2^{-a}
 * \frac{1 + 2z^{-1} + 1z^{-2}}{1 + (-2 + 2^{-b})z^{-1} + (1-2^{-b} + 4*
 * 2^{-a})z^{-2}} \f]
 *
 *  - The DC gain is exactly 1.
 *  - For real poles, and therefore no overshoot, use \f$ a <= 2b+4 \f$.
 *  - To calculate the poles, you can use this [wolfram alpha
 *  link](http://www.wolframalpha.com/input/?i=solve+%281++%2B+%28-2+%2B+2^-b%29z^-1++%2B+%281-2^-b+%2B+4*+2^-a%29z^-2%29+%3D+0+where+a+%3D+24+and+b+%3D+10)
 *  - The filter has a zero at \f$z = -1\f$
 *  - For \f$a=2b+4\f$, it has a pole at \f$z = \frac{(2^{b+1}-1)}{2^{b+1}}\f$
 *
 * 	Here are the specifications for a single stage filter, for values
 * \f$a=2b+4\f$ The delay time is the time it takes to rise to 0.5 in a step
 * response. When cascaded filters are used, multiply the delay time by the
 * number of cascades.
 *
 *  | a | b | Delay Time |
 *  |---|---|------------|
 * 	| 4 |	0 |  3 |
 * 	| 6 | 1 |  6 |
 * 	| 8 |	2 | 13 |
 * 	| 10 | 3 | 26 |
 * 	| 12 | 4 | 53 |
 * 	| 14 | 5 | 106 |
 * 	| 16 | 6 | 213 |
 *
 * The coefficients are powers of two (the original version used bit shifts on
 * integers), here they are doubles so nothing is rounded away.
 */
class FilterSection {
 public:
  FilterSection() { setCoefficients(2); }

  void init(temperature val) {
    for (int i = 0; i < 3; i++) {
      _xv[i] = val;
      _yv[i] = val;
    }
  }

  // The b value, a is calculated from it (a = 2b + 4)
  void setCoefficients(uint8_t bValue) {
    _kb = ldexp(1.0, -bValue);
    _ka = ldexp(1.0, -(2 * bValue + 4));
  }

  temperature add(temperature val) {
    _xv[2] = _xv[1];
    _xv[1] = _xv[0];
    _xv[0] = val;

    _yv[2] = _yv[1];
    _yv[1] = _yv[0];

    _yv[0] = 2 * _yv[1] - _yv[2] - _kb * (_yv[1] - _yv[2]) +
             _ka * (_xv[0] + 2 * _xv[1] + _xv[2]) - 4 * _ka * _yv[2];
    return _yv[0];
  }

  temperature readOutput() const { return _yv[0]; }
  temperature readInput() const { return _xv[0]; }

  // The output has just passed a top or a bottom, NAN if it has not
  temperature detectPosPeak() const {
    return (_yv[0] < _yv[1] && _yv[1] >= _yv[2]) ? _yv[1] : NAN;
  }
  temperature detectNegPeak() const {
    return (_yv[0] > _yv[1] && _yv[1] <= _yv[2]) ? _yv[1] : NAN;
  }

 private:
  // input and output history, newest first
  double _xv[3] = {0, 0, 0};
  double _yv[3] = {0, 0, 0};
  double _ka = 0;  // 2^-a
  double _kb = 0;  // 2^-b
};

constexpr auto numFilterSections = 3;

class CascadedFilter {
 public:
  /**
   * Three filter sections gives excellent filtering, without adding too
   * much delay.  For 3 sections the stop band attenuation is 3x the single
   * section attenuation in dB.  The delay is also tripled.
   */
  CascadedFilter() {}

  void init(temperature val) {
    for (auto& section : sections) section.init(val);
  }

  void setCoefficients(uint8_t bValue) {
    for (auto& section : sections) section.setCoefficients(bValue);
  }

  temperature add(temperature val) {
    for (auto& section : sections) val = section.add(val);
    return val;
  }

  temperature readOutput() const {
    return sections[numFilterSections - 1].readOutput();
  }
  temperature detectPosPeak() const {
    return sections[numFilterSections - 1].detectPosPeak();
  }
  temperature detectNegPeak() const {
    return sections[numFilterSections - 1].detectNegPeak();
  }

 private:
  FilterSection sections[numFilterSections];
};

#endif  // SRC_FILTERCASCADED_HPP_
