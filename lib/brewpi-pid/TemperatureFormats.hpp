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
#ifndef SRC_TEMPERATUREFORMATS_HPP_
#define SRC_TEMPERATUREFORMATS_HPP_

#include <math.h>
#include <stdint.h>

/**
 * Temperature values
 *
 * - A temperature, and a temperature difference, is a double in degrees
 * Celsius. Everything the controller handles is in Celsius, conversion to
 * other units (Fahrenheit) is done where a value is displayed or entered.
 * - NAN means that there is no value (no sensor reading, no target). Test for
 * it with isnan(), never with ==.
 */
typedef double temperature;

/**
 * Round a value to numDecimals (1, 2 or 3, anything else gives 3), half away
 * from zero. Used for JSON and logging. NAN (no value) gives 0.
 */
double roundTo(double value, uint8_t numDecimals);

#endif  // SRC_TEMPERATUREFORMATS_HPP_
