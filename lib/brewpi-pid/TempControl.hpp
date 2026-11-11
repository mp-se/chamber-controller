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
constexpr uint8_t COOLING_SAFETY_COOLDOWN = 9;  //!< Max cooling time reached
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
#include <ActuatorValue.hpp>
#include <TempSensor.hpp>
#include <TempSensorDisconnected.hpp>

extern ValueActuator defaultActuator;
extern DisconnectedTempSensor defaultTempSensor;
extern TempSensor defaultBeerSensor;
extern TempSensor defaultFridgeSensor;

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

  void setHeatingActuator(Actuator* actuator) {
    Log.verbose(F("BREW: TempControl new heatActuator %x" CR), actuator);
    _heater = actuator;
  }
  const Actuator* getHeatingActuator() { return _heater; }
  bool isDefaultHeatingActator() { return _heater == &defaultActuator; }

  void setCoolingActuator(Actuator* actuator) {
    Log.verbose(F("BREW: TempControl new coolActuator %x" CR), actuator);
    _cooler = actuator;
  }
  const Actuator* getCoolingActuator() { return _cooler; }
  bool isDefaultCoolingActator() { return _cooler == &defaultActuator; }

  void setFanActuator(Actuator* actuator) {
    Log.verbose(F("BREW: TempControl new fanActuator %x" CR), actuator);
    _fan = actuator;
  }
  const Actuator* getFanActuator() { return _fan; }
  bool isDefaultFanActator() { return _fan == &defaultActuator; }

  void setBeerSensor(TempSensor* sensor) {
    Log.verbose(F("BREW: TempControl new beerSensor %x" CR), sensor);
    _beerSensor = sensor;
    initFilters();
  }
  bool isDefaultBeerSensor() { return _beerSensor == &defaultBeerSensor; }
  const TempSensor* getBeerSensor() { return _beerSensor; }

  void setFridgeSensor(TempSensor* sensor) {
    Log.verbose(F("BREW: TempControl new fridgeSensor %x" CR), sensor);
    _fridgeSensor = sensor;
    initFilters();
  }
  bool isDefaultFridgeSensor() { return _fridgeSensor == &defaultFridgeSensor; }
  const TempSensor* getFridgeSensor() { return _fridgeSensor; }

  // Status methods
  uint8_t getState() { return _state; }
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
      case ControllerState::COOLING_SAFETY_COOLDOWN:
        return "Cooling safety cooldown";
    }
    return "Unknown state";
  }

  uint16_t getWaitTime() { return _waitTime; }
  ticks_seconds_t timeSinceCooling();
  ticks_seconds_t timeSinceHeating();
  ticks_seconds_t timeSinceIdle();

  bool stateIsCooling();
  bool stateIsHeating();
  bool modeIsBeer() { return (_cs.mode == ControllerMode::beerConstant); }

  // Public functions for the outside: Celsius rounded to the number of decimals
  // used for display, 0 when there is no value (no sensor, no target)
  double getBeerTemperature() {
    return roundTo(getBeerTemp(), Config::TempFormat::tempDecimals);
  }
  double getBeerTemperatureSetting() {
    return roundTo(getBeerSetting(), Config::TempFormat::tempDecimals);
  }
  double getFridgeTemperature() {
    return roundTo(getFridgeTemp(), Config::TempFormat::tempDecimals);
  }
  double getFridgeTemperatureSetting() {
    return roundTo(getFridgeSetting(), Config::TempFormat::tempDecimals);
  }
  void setBeerTargetTemperature(double newTemp) {
    Log.info(F("BREW: Setting new beer target temperature to %F" CR), newTemp);

    setBeerTemp(newTemp);
  }
  void setFridgeTargetTemperature(double newTemp) {
    Log.info(F("BREW: Setting new fridge target temperature to %F" CR),
             newTemp);

    setFridgeTemp(newTemp);
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

  // The control logic needs a beer and a fridge temperature. With a single probe that probe is both, so
  // the device works with either probe on its own. The getters above and the display keep showing only
  // the probes that exist.
  TempSensor* beerControl() { return isDefaultBeerSensor() ? _fridgeSensor : _beerSensor; }
  TempSensor* fridgeControl() { return isDefaultFridgeSensor() ? _beerSensor : _fridgeSensor; }

  // Sensors and Actuators
  TempSensor* _beerSensor;    //!< Temp sensor monitoring beer
  TempSensor* _fridgeSensor;  //!< Temp sensor monitoring fridge
  Actuator* _heater;          //!< Actuator used to call for heat
  Actuator* _cooler;          //!< Actuator used to call for cool
  Actuator* _fan;             //!< Actuator used to run the fan
  bool _beerLost = false;     //!< Beer mode, but the beer sensor gives no data
  bool _pidValid = false;     //!< The PID has run, its integral term is meaningful

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

  void updateWaitTime(uint16_t newTimeLimit, ticks_seconds_t newTimeSince) {
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
                           ticks_seconds_t sinceIdle);

  void updateSensor(TempSensor* sensor);

  temperature _storedBeerSetting = NAN;

  // Timers, the timestamps are millis() values (see HardwareTicks::secondsSince)
  ticks_millis_t _lastIdleTime;  //!< Last time the controller was idle
  ticks_millis_t _lastHeatTime;  //!< Last time that the controller was heating
  ticks_millis_t _lastCoolTime;  //!< Last time that the controller was cooling
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
