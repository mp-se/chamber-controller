/*
 * Chamber Controller
 * Copyright (c) 2024-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifndef SRC_PIDCONFIG_HPP_
#define SRC_PIDCONFIG_HPP_

#include <TempControl.hpp>
#include <baseconfig.hpp>

class PidConfig : public BaseConfig {
 private:
  // PID controller settings
  char _controllerMode = ControllerMode::off;
  String _fridgeSensorId = "";
  String _beerSensorId = "";
  String _beerBleSensorId = "";
  float _targetTemperature = 5;
  float _fridgeSensorOffset = 0.0;
  float _beerSensorOffset = 0.0;
  int _restartInterval = 60 * 4;  // in minutes
  bool _enableCooling = false;
  bool _enableHeating = false;
  bool _enableFan = false;
  bool _invertPins = false;

  // BLE settings
  bool _enableBlePush = false;
  bool _enableBleScan = false;
  bool _bleActiveScan = false;
  int _bleScanTime = 3;
  int _bleSensorValidTime = 15;

  // Params for remote control via brewlogger
  bool _remoteControlActive = false;
  String _remotePreviousBleSensorId = "";
  char _remotePreviousMode = ControllerMode::off;
  float _remotePreviousTargetTemp = 5.0;

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

  bool isFanEnabled() const { return _enableFan; }
  void setFanEnabled(bool b) {
    _enableFan = b;
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

  bool getRemoteControlActive() const { return _remoteControlActive; }
  void setRemoteControlActive(bool b) {
    _remoteControlActive = b;
    _saveNeeded = true;
  }

  const char* getRemotePreviousBleSensorId() const {
    return _remotePreviousBleSensorId.c_str();
  }
  void setRemotePreviousBleSensorId(String s) {
    _remotePreviousBleSensorId = s;
    _saveNeeded = true;
  }

  char getRemotePreviousMode() const { return _remotePreviousMode; }
  void setRemotePreviousMode(char c) {
    _remotePreviousMode = c;
    _saveNeeded = true;
  }

  float getRemotePreviousTargetTemp() const {
    return _remotePreviousTargetTemp;
  }
  void setRemotePreviousTargetTemp(float f) {
    _remotePreviousTargetTemp = f;
    _saveNeeded = true;
  }

  void createJson(JsonObject& doc) const override;
  void parseJson(JsonObject& doc) override;
};

extern PidConfig myConfig;

#endif  // SRC_PIDCONFIG_HPP_

// EOF
