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
#ifndef SRC_TEMPCONTROL_HPP_
#define SRC_TEMPCONTROL_HPP_

// #include <ArduinoJson.h>

#include <Actuator.hpp>
#include <ActuatorAutoOff.hpp>
#include <ActuatorValue.hpp>
#include <Config.hpp>
#include <ControlConstants.hpp>
#include <ControlSettings.hpp>
#include <ControlVariables.hpp>
#include <MinTimes.hpp>
#include <TempSensor.hpp>
#include <TemperatureFormats.hpp>

namespace ControllerMode {
constexpr const char fridgeConstant = 'f';
constexpr const char beerConstant = 'b';
constexpr const char off = 'o';
constexpr const char test = 't';
};  // namespace ControllerMode

namespace ControllerState {
constexpr uint8_t IDLE = 0;             //!< Neither heating, nor cooling
constexpr uint8_t STATE_OFF = 1;        //!< Disabled
constexpr uint8_t HEATING = 2;          //!< Calling for heat
constexpr uint8_t COOLING = 3;          //!< Calling for cool
constexpr uint8_t WAITING_TO_COOL = 4;  //!< Waiting to cool. (Compressor delay)
constexpr uint8_t WAITING_TO_HEAT = 5;  //!< Waiting to heat. (Compressor delay)
constexpr uint8_t WAITING_FOR_PEAK_DETECT = 6;  //!< Waiting for peak detection
constexpr uint8_t COOLING_MIN_TIME = 7;         // 8
constexpr uint8_t HEATING_MIN_TIME = 8;         // 9
};  // namespace ControllerState

extern ValueActuator defaultActuator;

/**
 * Temperature control PID implmentation
 *
 * This is the heart of the brewpi system.  It handles turning on and off heat
 * & cool to track a target temperature.
 *
 * Temp Control tracking can be done using several different modes
 *
 * - Beer: Heat & Cool are applied to keep a probe in the fermenting beer at a
 * target.
 * - Fridge: Heat & Cool are applied to keep a probe in the chamber surrounding
 * the beer at a target.
 */
class TempControl {
 public:
  TempControl();
  ~TempControl();

  // Control methods
  void init(MinTimesSettingsChoice choise = MIN_TIMES_DEFAULT);
  void reset();
  void loop();

  void setMode(char newMode, bool force = false);
  char getMode() { return _cs.mode; }

  void setHeatActuator(Actuator* actuator) {
    Log.verbose(F("BREW: TempControl new heatActuator %x" CR), actuator);
    _heater = actuator;
  }

  void setCoolActuator(Actuator* actuator) {
    Log.verbose(F("BREW: TempControl new coolActuator %x" CR), actuator);
    _cooler = actuator;
  }

  void setBeerSensor(TempSensor* sensor) {
    Log.verbose(F("BREW: TempControl new beerSensor %x" CR), sensor);
    _beerSensor = sensor;
  }

  void setFridgeSensor(TempSensor* sensor) {
    Log.verbose(F("BREW: TempControl new fridgeSensor %x" CR), sensor);
    _fridgeSensor = sensor;
  }

  // Status methods
  unsigned char getState() { return _state; }
  const char* getStateAsString() {
    switch (_state) {
      case ControllerState::IDLE:
        return "Idle";
      case ControllerState::STATE_OFF:
        return "Off";
      case ControllerState::HEATING:
        return "Heating";
      case ControllerState::COOLING:
        return "Cooling";
      case ControllerState::WAITING_TO_COOL:
        return "Waiting to cool";
      case ControllerState::WAITING_TO_HEAT:
        return "Waiting to heat";
      case ControllerState::WAITING_FOR_PEAK_DETECT:
        return "Waiting for peak detect";
      case ControllerState::COOLING_MIN_TIME:
        return "Cooling min time";
      case ControllerState::HEATING_MIN_TIME:
        return "Heating min time";
    }
    return "Unknown state";
  }

  uint16_t getWaitTime() { return _waitTime; }
  uint16_t timeSinceCooling();
  uint16_t timeSinceHeating();
  uint16_t timeSinceIdle();

  bool stateIsCooling();
  bool stateIsHeating();
  bool modeIsBeer() { return (_cs.mode == ControllerMode::beerConstant); }

  // Public function to ensure we map temperatures between human readable and
  // internal temperature formats
  double getBeerTemperature() {
    return tempToDouble(getBeerTemp(), Config::TempFormat::tempDecimals);
  }
  double getBeerTemperatureSetting() {
    return tempToDouble(getBeerSetting(), Config::TempFormat::tempDecimals);
  }
  double getFridgeTemperature() {
    return tempToDouble(getFridgeTemp(), Config::TempFormat::tempDecimals);
  }
  double getFridgeTemperatureSetting() {
    return tempToDouble(getFridgeSetting(), Config::TempFormat::tempDecimals);
  }
  void setBeerTargetTemperature(double newTemp) {
    setBeerTemp(doubleToTemp(newTemp));
  }
  void setFridgeTargetTemperature(double newTemp) {
    setFridgeTemp(doubleToTemp(newTemp));
  }

  // Public function to gain insights into the internal data
  const ControlConstants& getControlConstants() { return _cc; }
  const ControlSettings& getControlSettings() { return _cs; }
  const ControlVariables& getControlVariables() { return _cv; }
  const MinTimes& getMinTimes() { return _minTimes; }

  // Load from disk
  void loadSettings();
  void loadConstants();

 private:
  void storeSettings();
  void loadDefaultSettings();

  void storeConstants();
  void loadDefaultConstants();

  temperature getBeerTemp();
  temperature getBeerSetting();
  void setBeerTemp(temperature newTemp);

  temperature getFridgeTemp();
  temperature getFridgeSetting();
  void setFridgeTemp(temperature newTemp);

  // Sensors and Actuators
  TempSensor* _beerSensor;    //!< Temp sensor monitoring beer
  TempSensor* _fridgeSensor;  //!< Temp sensor monitoring fridge
  Actuator* _heater;          //!< Actuator used to call for heat
  Actuator* _cooler;          //!< Actuator used to call for cool

  // Control parameters
  ControlConstants _cc;
  ControlSettings _cs;
  ControlVariables _cv;
  MinTimes _minTimes;

  void updateTemperatures();
  void updatePID();
  void updateState();
  void updateOutputs();
  void detectPeaks();

  void resetWaitTime() { _waitTime = 0; }

  void updateWaitTime(uint16_t newTimeLimit, uint16_t newTimeSince) {
    if (newTimeSince < newTimeLimit) {
      uint16_t newWaitTime = newTimeLimit - newTimeSince;
      if (newWaitTime > _waitTime) {
        _waitTime = newWaitTime;
      }
    }
  }

  void initFilters();

  void increaseEstimator(temperature* estimator, temperature error);
  void decreaseEstimator(temperature* estimator, temperature error);

  void updateEstimatedPeak(uint16_t estimate, temperature estimator,
                           uint16_t sinceIdle);

  void updateSensor(TempSensor* sensor);

  temperature _storedBeerSetting;

  // Timers
  uint16_t _lastIdleTime;  //!< Last time the controller was idle
  uint16_t _lastHeatTime;  //!< Last time that the controller was heating
  uint16_t _lastCoolTime;  //!< Last time that the controller was cooling
  uint16_t _waitTime;

  // State variables
  uint8_t _state;         //!< Current controller state
  bool _doPosPeakDetect;  //!< True if the controller is doing
                          //!< positive peak detection
  bool _doNegPeakDetect;  //!< True if the controller is doing
                          //!< negative peak detection
};

extern TempControl tempControl;

#endif  // SRC_TEMPCONTROL_HPP_

// EOF
