/*
MIT License

Copyright (c) 2024-2025 Magnus

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
  String _beerBleSensorId = "";
  char _controllerMode = ControllerMode::off;
  float _targetTemperature = 5;
  float _fridgeSensorOffset = 0.0;
  float _beerSensorOffset = 0.0;
  int _restartInterval = 60 * 4;  // in minutes
  bool _enableCooling = false;
  bool _enableHeating = false;
  bool _invertPins = false;
  bool _enableBlePush = false;
  bool _enableBleScan = false;
  bool _bleActiveScan = false;  // This is a constant for now
  int _bleScanTime = 3;         // This is a constant for now
  int _bleSensorValidTime = 15;

 public:
  PidConfig(String baseMDNS, String fileName);

  const char* getFridgeSensorId() const { return _fridgeSensorId.c_str(); }
  void setFridgeSensorId(String s) {
    _fridgeSensorId = s;
    _saveNeeded = true;
  }
  bool isFridgeSensorEnabled() const { return _fridgeSensorId.length() != 0; }

  const char* getBeerSensorId() const { return _beerSensorId.c_str(); }
  void setBeerSensorId(String s) {
    _beerSensorId = s;
    _saveNeeded = true;
  }
  bool isBeerSensorEnabled() const { return _beerSensorId.length() != 0; }

  const char* getBeerBleSensorId() const { return _beerBleSensorId.c_str(); }
  void setBeerBleSensorId(String s) {
    _beerBleSensorId = s;
    _saveNeeded = true;
  }
  bool isBeerBleSensorEnabled() const { return _beerBleSensorId.length() != 0; }

  float getFridgeSensorOffset() const { return _fridgeSensorOffset; }
  void setFridgeSensorOffset(float t) {
    _fridgeSensorOffset = t;
    _saveNeeded = true;
  }

  float getBeerSensorOffset() const { return _beerSensorOffset; }
  void setBeerSensorOffset(float t) {
    _beerSensorOffset = t;
    _saveNeeded = true;
  }

  float getTargetTemperature() const { return _targetTemperature; }
  void setTargetTemperature(float v) {
    _targetTemperature = v;
    _saveNeeded = true;
  }

  bool isCoolingEnabled() const { return _enableCooling; }
  void setCoolingEnabled(bool b) {
    _enableCooling = b;
    _saveNeeded = true;
  }

  bool isHeatingEnabled() const { return _enableHeating; }
  void setHeatingEnabled(bool b) {
    _enableHeating = b;
    _saveNeeded = true;
  }

  bool isPinsInverted() const { return _invertPins; }
  void setPinsInverted(bool b) {
    _invertPins = b;
    _saveNeeded = true;
  }

  bool isBlePushEnabled() const { return _enableBlePush; }
  void setBlePushEnabled(bool b) {
    _enableBlePush = b;
    _saveNeeded = true;
  }

  bool isBleScanEnabled() const { return _enableBleScan; }
  void setBleScanEnabled(bool b) {
    _enableBleScan = b;
    _saveNeeded = true;
  }

  int getRestartInterval() const { return _restartInterval; }
  void setRestartInterval(int v) {
    _restartInterval = v;
    _saveNeeded = true;
  }

  char getControllerMode() const { return _controllerMode; }
  void setControllerMode(char c) {
    if (c == ControllerMode::off || c == ControllerMode::beerConstant ||
        c == ControllerMode::fridgeConstant) {
      _controllerMode = c;
      _saveNeeded = true;
    }
  }
  bool isControllerFridgeConstant() const {
    return _controllerMode == ControllerMode::fridgeConstant;
  }
  bool isControllerBeerConstant() const {
    return _controllerMode == ControllerMode::beerConstant;
  }
  bool isControllerOff() const {
    return _controllerMode == ControllerMode::off;
  }

  int getBleScanTime() const { return _bleScanTime; }
  // void setBleScanTime(int v) {
  //   _bleScanTime = v;
  //   _saveNeeded = true;
  // }

  bool getBleActiveScan() const { return _bleActiveScan; }
  // void setBleActiveScan(bool b) {
  //   _bleActiveScan = b;
  //   _saveNeeded = true;
  // }

  int getBleSensorValidTime() const { return _bleSensorValidTime; }
  void setBleSensorValidTime(int v) {
    _bleSensorValidTime = v;
    _saveNeeded = true;
  }

  void createJson(JsonObject& doc) const override;
  void parseJson(JsonObject& doc) override;
};

extern PidConfig myConfig;

#endif  // SRC_PIDCONFIG_HPP_

// EOF
