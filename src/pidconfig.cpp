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
#include <ArduinoJson.h>

#include <espframework.hpp>
#include <main.hpp>
#include <pidconfig.hpp>

PidConfig::PidConfig(String baseMDNS, String fileName)
    : BaseConfig(baseMDNS, fileName) {}

void PidConfig::createJson(JsonObject& doc) {
  // Call base class functions
  createJsonBase(doc);
  createJsonWifi(doc);
  // createJsonOta(doc);
  createJsonPush(doc);

  // Handle project specific config
  doc[PARAM_FRIDGE_SENSOR_ID] = getFridgeSensorId();
  doc[PARAM_BEER_SENSOR_ID] = getBeerSensorId();

  doc[PARAM_CONTROLLER_MODE] = String(getControllerMode());
  doc[PARAM_TARGET_TEMPERATURE] = getTargetTemperature();
  doc[PARAM_ENABLE_COOLING] = isCoolingEnabled();
  doc[PARAM_ENABLE_HEATING] = isHeatingEnabled();
  doc[PARAM_INVERT_PINS] = isPinsInverted();
}

void PidConfig::parseJson(JsonObject& doc) {
  // Call base class functions
  parseJsonBase(doc);
  parseJsonWifi(doc);
  // parseJsonOta(doc);
  parseJsonPush(doc);

  // Handle project specific config
  if (!doc[PARAM_CONTROLLER_MODE].isNull()) {
    String s = doc[PARAM_CONTROLLER_MODE];
    setControllerMode(s.charAt(0));
  }
  if (!doc[PARAM_FRIDGE_SENSOR_ID].isNull())
    setFridgeSensorId(doc[PARAM_FRIDGE_SENSOR_ID]);
  if (!doc[PARAM_BEER_SENSOR_ID].isNull())
    setBeerSensorId(doc[PARAM_BEER_SENSOR_ID]);
  if (!doc[PARAM_TARGET_TEMPERATURE].isNull())
    setTargetTemperature(doc[PARAM_TARGET_TEMPERATURE].as<float>());
  if (!doc[PARAM_ENABLE_COOLING].isNull())
    setCoolingEnabled(doc[PARAM_ENABLE_COOLING].as<bool>());
  if (!doc[PARAM_ENABLE_HEATING].isNull())
    setHeatingEnabled(doc[PARAM_ENABLE_HEATING].as<bool>());
  if (!doc[PARAM_INVERT_PINS].isNull())
    setPinsInverted(doc[PARAM_INVERT_PINS].as<bool>());
}

// EOF
