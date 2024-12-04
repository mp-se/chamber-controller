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
#ifndef SRC_FILTERFIXED_HPP_
#define SRC_FILTERFIXED_HPP_

#include <TemperatureFormats.hpp>
#include <log.hpp>

/**
 * This class implements an IIR low pass filter, with the following frequency
 * response
 *
 * \f[
 * H(z) = 2^{-a}
 * \frac{1 + 2z^{-1} + 1z^{-2}}{1 + (-2 + 2^{-b})z^{-1} + (1-2^{-b} + 4*
 * 2^{-a})z^{-2}} \f]
 *
 *  - All filter coefficients are powers of two, so the filter can be
 * efficiently implemented with bit shifts
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
 *  Use this MATLAB script to visualize the filter:
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 	NUM_SECTIONS=1;
 * 	a=6; b=1; FS=1;
 * 	DEN = [1  , -2 + 2^-b  , 1-2^-b+ 4*2^-a];
 * 	NUM = 2^(-a)*[1, 2, 1];
 * 	F=dfilt.df2(NUM,DEN);
 * 	H=F;
 * 	for i=2:NUM_SECTIONS
 * 	H=dfilt.cascade(H,F);
 * 	end
 * 	%H=F^NUM_SECTIONS;
 * 	h=fvtool(H,'Fs',FS)
 * 	zeropos = roots(NUM)
 * 	polepos = roots(DEN)
 * 	set(h,'FrequencyRange', 'Specify freq. vector');
 * 	set(h,'FrequencyScale','Log')
 * 	set(h,'FrequencyVector', logspace(-4,0,1000));
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
class FixedFilter {
 public:
  // input and output arrays
  temperature_precise _xv[3];
  temperature_precise _yv[3];
  uint8_t _a;
  uint8_t _b;

 public:
  FixedFilter() {
    Log.verbose(F("BREW: Creating FixedFilter." CR));

    setCoefficients(20); /* default to a b value of 2 */
  }
  ~FixedFilter() {}
  void init(temperature val);

  void setCoefficients(uint8_t bValue) {
    Log.verbose(F("BREW: Setting coefficients on FixedFilter to %d." CR),
    bValue);

    _a = bValue * 2 + 4;
    _b = bValue;
  }

  temperature add(temperature val);
  temperature_precise addDoublePrecision(temperature_precise val);
  temperature readOutput() { return _yv[0] >> 16; }
  temperature readInput() { return _xv[0] >> 16; }
  temperature_precise readOutputDoublePrecision() { return _yv[0]; }
  temperature_precise readPrevOutputDoublePrecision() { return _yv[1]; }
  temperature detectPosPeak();
  temperature detectNegPeak();
};

#endif  // SRC_FILTERFIXED_HPP_

// EOF
