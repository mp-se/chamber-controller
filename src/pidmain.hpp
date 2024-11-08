/*
MIT License

Copyright (c) 2024 Magnus

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef SRC_PIDMAIN_HPP_
#define SRC_PIDMAIN_HPP_

void configureTempControl();

constexpr auto PARAM_FRIDGE_SENSOR_ID = "fride_sensor_id";
constexpr auto PARAM_BEER_SENSOR_ID = "beer_sensor_id";
constexpr auto PARAM_CONTROLLER_MODE = "controller_mode";
constexpr auto PARAM_TARGET_TEMPERATURE = "target_temperature";
constexpr auto PARAM_ENABLE_COOLING = "enable_cooling";
constexpr auto PARAM_ENABLE_HEATING = "enable_heating";

#endif  // SRC_PIDMAIN_HPP_