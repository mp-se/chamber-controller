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
#ifndef SRC_PIDCONFIG_HPP_
#define SRC_PIDCONFIG_HPP_

#include <TempControl.hpp>
#include <baseconfig.hpp>

class PidConfig : public BaseConfig {
 private:
  String _fridgeSensorId = "";
  String _beerSensorId = "";
  char _controllerMode = ControllerMode::off;
  float _targetTemperature = 5;
  bool _enableCooling = false;
  bool _enableHeating = false;
  bool _invertPins = true;
  int _restartInterval = 60 * 4; // minutes 

  // TODO: Add option to define which MIN_TIMES profile to use.
  // TODO: Add offset for beer and fridge sensors

 public:
  PidConfig(String baseMDNS, String fileName);

  const char* getFridgeSensorId() { return _fridgeSensorId.c_str(); }
  void setFridgeSensorId(String s) {
    _fridgeSensorId = s;
    _saveNeeded = true;
  }
  bool isFridgeSensorEnabled() { return _fridgeSensorId.length() != 0; }

  const char* getBeerSensorId() { return _beerSensorId.c_str(); }
  void setBeerSensorId(String s) {
    _beerSensorId = s;
    _saveNeeded = true;
  }
  bool isBeerSensorEnabled() { return _beerSensorId.length() != 0; }

  float getTargetTemperature() { return _targetTemperature; }
  void setTargetTemperature(float v) {
    _targetTemperature = v;
    _saveNeeded = true;
  }

  bool isCoolingEnabled() { return _enableCooling; }
  void setCoolingEnabled(bool b) {
    _enableCooling = b;
    _saveNeeded = true;
  }

  bool isHeatingEnabled() { return _enableHeating; }
  void setHeatingEnabled(bool b) {
    _enableHeating = b;
    _saveNeeded = true;
  }

  bool isPinsInverted() { return _invertPins; }
  void setPinsInverted(bool b) {
    _invertPins = b;
    _saveNeeded = true;
  }

  int getRestartInterval() { return _restartInterval; }
  void setRestartInterval(int v) {
    _restartInterval = v;
    _saveNeeded = true;
  }

  char getControllerMode() { return _controllerMode; }
  void setControllerMode(char c) {
    if (c == ControllerMode::off || c == ControllerMode::beerConstant ||
        c == ControllerMode::fridgeConstant) {
      _controllerMode = c;
      _saveNeeded = true;
    }
  }
  bool isControllerFridgeConstant() {
    return _controllerMode == ControllerMode::fridgeConstant;
  }
  bool isControllerBeerConstant() {
    return _controllerMode == ControllerMode::beerConstant;
  }
  bool isControllerOff() { return _controllerMode == ControllerMode::off; }

  void createJson(JsonObject& doc);
  void parseJson(JsonObject& doc);
};

extern PidConfig myConfig;

#endif  // SRC_PIDCONFIG_HPP_

// EOF
