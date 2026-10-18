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

#include <limits.h>

#include <Config.hpp>
#include <MinTimes.hpp>
#include <TempControl.hpp>
#include <TemperatureFormats.hpp>
#include <Ticks.hpp>
#include <log.hpp>

TempControl tempControl;
ValueActuator defaultActuator;
DisconnectedTempSensor defaultTempSensor;
TempSensor defaultBeerSensor(TEMP_SENSOR_TYPE_BEER, &defaultTempSensor);
TempSensor defaultFridgeSensor(TEMP_SENSOR_TYPE_FRIDGE, &defaultTempSensor);

#ifndef min
#define min _min
#endif

#ifndef max
#define max _max
#endif

// Half of the precision of the sensors (1/16 degree). A beer this close to the
// target counts as being on the target ("1/2 sensor bit idle zone").
constexpr temperature HALF_SENSOR_BIT = 1.0 / 32;

TempControl::TempControl() {}

TempControl::~TempControl() {}

void TempControl::loop() {
  tempControl.updateTemperatures();
  tempControl.detectPeaks();
  tempControl.updatePID();
  tempControl.updateState();
  tempControl.updateOutputs();
}

void TempControl::init(MinTimesSettingsChoice choice) {
  Log.verbose(F("BREW: Initialize TempControl using MinTime=%d" CR), choice);

  _minTimes.setDefaults(choice);
#if defined(DEV_TESTING)
  _minTimes.setDefaults(MIN_TIMES_DEVELOP);
#endif

  _beerSensor = &defaultBeerSensor;
  _beerSensor->init();

  _fridgeSensor = &defaultFridgeSensor;
  _fridgeSensor->init();

  _heater = &defaultActuator;
  _cooler = &defaultActuator;
  _fan = &defaultActuator;

  loadDefaultConstants();
  loadDefaultSettings();
  initFilters();

  _state = ControllerState::IDLE;
  _beerLost = false;
  _pidValid = false;
  _cs.mode = ControllerMode::off;

  updateTemperatures();
  reset();

  // Do not allow heating/cooling directly after reset.
  // Could damage the compressor. For test purposes, set these to
  // ticks.millis() - 3600000 to eliminate waiting after reset.
  // init() is also called when the sensors or settings are reconfigured while
  // running, so this has to be relative to now and not to boot.
  _lastHeatTime = ticks.millis();
  _lastCoolTime = ticks.millis();
}

void TempControl::reset() {
  Log.verbose(F("BREW: Reset TempControl" CR));

  _doPosPeakDetect = false;
  _doNegPeakDetect = false;
}

void TempControl::updateSensor(TempSensor* sensor) {
  if (sensor == NULL) {
    Log.error(F("BREW: TempControl tried to update a sensor that is NULL" CR));
    return;
  }

  if (sensor == &defaultBeerSensor || sensor == &defaultFridgeSensor) {
    Log.verbose(F("BREW: Default sensor, no action required." CR));
    return;
  }

  sensor->update();
  if (!sensor->isConnected()) {
    Log.warning(F(
        "BREW: TempControl sensor is not connected, trying to initialize." CR));
    sensor->init();
  }
}

void TempControl::updateTemperatures() {
  Log.verbose(F("BREW: Updating temp sensors" CR));

  updateSensor(_beerSensor);
  updateSensor(_fridgeSensor);
}

void TempControl::updatePID() {
  Log.verbose(F("BREW: Updating PID for TempControl" CR));

  static unsigned char integralUpdateCounter = 0;
  if (modeIsBeer()) {
    Log.verbose(F("BREW: Updating PID, mode is beer." CR));

    if (isnan(_cs.beerSetting)) {
      // beer setting is not updated yet
      // set fridge to unknown too
      _cs.fridgeSetting = NAN;
      return;
    }

    // If either sensor is not connected, although the reads should still work
    // (since we're reading from filters) we probably don't want them.
    if (!beerControl()->isConnected() || !fridgeControl()->isConnected()) {
      // The question here is if we should reset the PID (or decrement some kind
      // of counter to reset the PID)
      Log.warning(
          F("BREW: No sensor is connected, unable to get temperatures." CR));

      // Without a beer temperature there is no PID. Control the fridge on the
      // steady state part of the last PID output, the beer target plus the
      // integral term (updateState() does the rest) until the beer sensor is
      // back. P and D are momentary values, they are not kept. The integral
      // term is 0 until the PID has run.
      if (fridgeControl()->isConnected()) {
        _cs.fridgeSetting =
            constrain(_cs.beerSetting + (_pidValid ? _cv.i : 0.0),
                      _cc.tempSettingMin, _cc.tempSettingMax);
      }
      return;
    }

    // fridge setting is calculated with PID algorithm. Beer temperature error
    // is input to PID
    _cv.beerDiff = _cs.beerSetting - beerControl()->readSlowFiltered();
    _cv.beerSlope = beerControl()->readSlope();
    temperature fridgeFastFiltered = fridgeControl()->readFastFiltered();

    if (integralUpdateCounter++ == 60) {
      integralUpdateCounter = 0;

      temperature integratorUpdate = _cv.beerDiff;

      // Only update integrator in IDLE, because thats when the fridge temp has
      // reached the fridge setting. If the beer temp is still not correct, the
      // fridge setting is too low/high and integrator action is needed.
      if (_state != ControllerState::IDLE) {
        integratorUpdate = 0;
      } else if (fabs(integratorUpdate) < _cc.iMaxError) {
        // difference is smaller than iMaxError
        // check additional conditions to see if integrator should be active to
        // prevent windup
        bool updateSign = (integratorUpdate > 0);  // 1 = positive, 0 = negative
        bool integratorSign = (_cv.diffIntegral > 0);

        if (updateSign == integratorSign) {
          // beerDiff and integrator have same sign. Integrator would be
          // increased.

          // If actuator is already at max increasing actuator will only cause
          // integrator windup.
          integratorUpdate =
              (_cs.fridgeSetting >= _cc.tempSettingMax) ? 0 : integratorUpdate;
          integratorUpdate =
              (_cs.fridgeSetting <= _cc.tempSettingMin) ? 0 : integratorUpdate;
          integratorUpdate =
              ((_cs.fridgeSetting - _cs.beerSetting) >= _cc.pidMax)
                  ? 0
                  : integratorUpdate;
          integratorUpdate =
              ((_cs.beerSetting - _cs.fridgeSetting) >= _cc.pidMax)
                  ? 0
                  : integratorUpdate;

          // cooling and fridge temp is more than 2 degrees from setting,
          // actuator is saturated.
          integratorUpdate =
              (!updateSign && (fridgeFastFiltered > (_cs.fridgeSetting + 2)))
                  ? 0
                  : integratorUpdate;

          // heating and fridge temp is more than 2 degrees from setting,
          // actuator is saturated.
          integratorUpdate =
              (updateSign && (fridgeFastFiltered < (_cs.fridgeSetting - 2)))
                  ? 0
                  : integratorUpdate;
        } else {
          // integrator action is decreased. Decrease faster than increase.
          integratorUpdate = integratorUpdate * 2;
        }
      } else {
        // decrease integral by 1/8 when far from the end value to reset the
        // integrator
        integratorUpdate = -(_cv.diffIntegral / 8);
      }
      _cv.diffIntegral = _cv.diffIntegral + integratorUpdate;
    }

    // calculate PID parts
    _cv.p = _cc.Kp * _cv.beerDiff;
    _cv.i = _cc.Ki * _cv.diffIntegral;
    _cv.d = _cc.Kd * _cv.beerSlope;
    temperature newFridgeSetting = _cs.beerSetting;
    newFridgeSetting += _cv.p;
    newFridgeSetting += _cv.i;
    newFridgeSetting += _cv.d;

    // constrain to tempSettingMin or beerSetting - pidMax, whichever is lower.
    temperature lowerBound =
        (_cs.beerSetting <= _cc.tempSettingMin + _cc.pidMax)
            ? _cc.tempSettingMin
            : _cs.beerSetting - _cc.pidMax;
    // constrain to tempSettingMax or beerSetting + pidMax, whichever is higher.
    temperature upperBound =
        (_cs.beerSetting >= _cc.tempSettingMax - _cc.pidMax)
            ? _cc.tempSettingMax
            : _cs.beerSetting + _cc.pidMax;

    _cs.fridgeSetting = constrain(newFridgeSetting, lowerBound, upperBound);
    _pidValid = true;
  } else if (_cs.mode == ControllerMode::fridgeConstant) {
    // FridgeTemperature is set manually, use NAN to indicate beer temp
    // is not active
    _cs.beerSetting = NAN;
  }
}

void TempControl::updateState() {
  Log.verbose(F("BREW: Updating TempControl state" CR));

  // update state
  bool stayIdle = false;

  if (_cs.mode == ControllerMode::off) {
    _state = ControllerState::STATE_OFF;
    stayIdle = true;
  } else if (isnan(_cs.fridgeSetting) || !fridgeControl()->isConnected()) {
    // stay idle when the fridge sensor is disconnected, or the fridge
    // setting is NAN Of note - setting the mode to Modes::off also
    // sets cs.fridgeSetting to NAN
    _state = ControllerState::IDLE;
    stayIdle = true;
  }

  // In beer mode the beer temperature takes part in the decisions. If the beer
  // sensor gives no readings (a BLE sensor out of range) the fridge is
  // controlled on the fridge setting alone, like in fridge constant mode, until
  // the sensor is back.
  bool useBeer = _cs.mode != ControllerMode::fridgeConstant &&
                 beerControl()->isConnected();
  bool beerLost = modeIsBeer() && !beerControl()->isConnected();
  if (beerLost != _beerLost) {
    _beerLost = beerLost;
    if (beerLost) {
      Log.warning(
          F("BREW: Beer sensor has no data, controlling the fridge on the "
            "fridge setting %F." CR),
          _cs.fridgeSetting);
    } else {
      Log.info(F("BREW: Beer sensor is back, controlling on beer." CR));
    }
  }

  ticks_seconds_t sinceIdle = timeSinceIdle();
  ticks_seconds_t sinceCooling = timeSinceCooling();
  ticks_seconds_t sinceHeating = timeSinceHeating();
  temperature fridgeFast = fridgeControl()->readFastFiltered();
  temperature beerFast = beerControl()->readFastFiltered();
  ticks_millis_t now = ticks.millis();
  switch (_state) {
    case ControllerState::IDLE:
    case ControllerState::STATE_OFF:
    case ControllerState::WAITING_TO_COOL:
    case ControllerState::WAITING_TO_HEAT:
    case ControllerState::WAITING_FOR_PEAK_DETECT: {
      _lastIdleTime = now;
      // set waitTime to zero. It will be set to the maximum required waitTime
      // below when wait is in effect.
      if (stayIdle) {
        break;
      }
      resetWaitTime();
      if (fridgeFast > (_cs.fridgeSetting +
                        _cc.idleRangeHigh)) {  // fridge temperature is too high
        updateWaitTime(_minTimes.MIN_SWITCH_TIME, sinceHeating);
        if (!useBeer) {
          updateWaitTime(_minTimes.MIN_COOL_OFF_TIME_FRIDGE_CONSTANT,
                         sinceCooling);
        } else {
          if (beerFast < (_cs.beerSetting + HALF_SENSOR_BIT)) {
            // If beer is already under target, stay/go to idle.
            _state = ControllerState::IDLE;  // beer is already colder than
                                             // setting, stay in or go to idle
            break;
          }
          updateWaitTime(_minTimes.MIN_COOL_OFF_TIME, sinceCooling);
        }
        if (_cooler != &defaultActuator) {
          if (getWaitTime() > 0) {
            _state = ControllerState::WAITING_TO_COOL;
          } else {
            _state = ControllerState::COOLING;
          }
        }
      } else if (fridgeFast <
                 (_cs.fridgeSetting +
                  _cc.idleRangeLow)) {  // fridge temperature is too low
        updateWaitTime(_minTimes.MIN_SWITCH_TIME, sinceCooling);
        updateWaitTime(_minTimes.MIN_HEAT_OFF_TIME, sinceHeating);
        if (useBeer) {
          if (beerFast > (_cs.beerSetting - HALF_SENSOR_BIT)) {
            // If beer is already over target, stay/go to idle.
            _state = ControllerState::IDLE;  // beer is already warmer than
                                             // setting, stay in or go to idle
            break;
          }
        }
        if (_heater != &defaultActuator) {
          if (getWaitTime() > 0) {
            _state = ControllerState::WAITING_TO_HEAT;
          } else {
            _state = ControllerState::HEATING;
          }
        }
      } else {
        _state = ControllerState::IDLE;  // within IDLE range, always go to IDLE
        break;
      }
      if (_state == ControllerState::HEATING ||
          _state == ControllerState::COOLING) {
        if (_doNegPeakDetect == true || _doPosPeakDetect == true) {
          // If peak detect is not finished, but the fridge wants to switch to
          // heat/cool Wait for peak detection and display 'Await peak detect'
          // on display
          _state = ControllerState::WAITING_FOR_PEAK_DETECT;
          break;
        }
      }
    } break;
    case ControllerState::COOLING:
    case ControllerState::COOLING_MIN_TIME: {
      _doNegPeakDetect = true;
      _lastCoolTime = now;
      updateEstimatedPeak(_cc.maxCoolTimeForEstimate, _cs.coolEstimator,
                          sinceIdle);
      _state =
          ControllerState::COOLING;  // set to cooling here, so the display of
                                     // COOLING/COOLING_MIN_TIME is correct

      if (sinceIdle >= _minTimes.MAX_COOL_ON_TIME) {
        Log.warning(F("BREW: Maximum cooling time reached; forcing compressor "
                      "cooldown." CR));
        _state = ControllerState::COOLING_SAFETY_COOLDOWN;
        break;
      }

      // stop cooling when estimated fridge temp peak lands on target or if beer
      // is already too cold (1/2 sensor bit idle zone)
      if (_cv.estimatedPeak <= _cs.fridgeSetting ||
          (useBeer && beerFast < (_cs.beerSetting - HALF_SENSOR_BIT))) {
        if (sinceIdle > _minTimes.MIN_COOL_ON_TIME) {
          _cv.negPeakEstimate =
              _cv.estimatedPeak;  // remember estimated peak when I switch to
                                  // IDLE, to adjust estimator later
          _state = ControllerState::IDLE;
          break;
        } else {
          _state = ControllerState::COOLING_MIN_TIME;
          break;
        }
      }
    } break;
    case ControllerState::COOLING_SAFETY_COOLDOWN: {
      // Keep the compressor off for the full safety cooldown even if the
      // chamber remains above its target. _lastCoolTime was updated while
      // cooling, so it is the start point for this enforced pause.
      resetWaitTime();
      updateWaitTime(_minTimes.COOLING_SAFETY_COOLDOWN, sinceCooling);
      if (getWaitTime() == 0) _state = ControllerState::IDLE;
    } break;
    case ControllerState::HEATING:
    case ControllerState::HEATING_MIN_TIME: {
      _doPosPeakDetect = true;
      _lastHeatTime = now;
      updateEstimatedPeak(_cc.maxHeatTimeForEstimate, _cs.heatEstimator,
                          sinceIdle);
      _state =
          ControllerState::HEATING;  // reset to heating here, so the display of
                                     // HEATING/HEATING_MIN_TIME is correct

      // stop heating when estimated fridge temp peak lands on target or if beer
      // is already too warm (1/2 sensor bit idle zone)
      if (_cv.estimatedPeak >= _cs.fridgeSetting ||
          (useBeer && beerFast > (_cs.beerSetting + HALF_SENSOR_BIT))) {
        if (sinceIdle > _minTimes.MIN_HEAT_ON_TIME) {
          _cv.posPeakEstimate =
              _cv.estimatedPeak;  // remember estimated peak when I switch to
                                  // IDLE, to adjust estimator later
          _state = ControllerState::IDLE;
          break;
        } else {
          _state = ControllerState::HEATING_MIN_TIME;
          break;
        }
      }
    } break;
  }
}

void TempControl::updateEstimatedPeak(uint16_t timeLimit, temperature estimator,
                                      ticks_seconds_t sinceIdle) {
  uint16_t activeTime = min(static_cast<ticks_seconds_t>(timeLimit),
                            sinceIdle);  // heat or cool time in seconds
  temperature estimatedOvershoot =
      estimator * activeTime /
      3600;  // overshoot estimator is in overshoot per hour
  if (stateIsCooling()) {
    estimatedOvershoot =
        -estimatedOvershoot;  // when cooling subtract overshoot from fridge
                              // temperature
  }
  _cv.estimatedPeak = fridgeControl()->readFastFiltered() + estimatedOvershoot;
}

void TempControl::updateOutputs() {
  if (_cs.mode == ControllerMode::test) return;

  Log.verbose(F("BREW: TempControl is updating outputs." CR));

  bool heating = stateIsHeating();
  bool cooling = stateIsCooling();
  _cooler->setActive(cooling);
  _heater->setActive(heating);
}

void TempControl::detectPeaks() {
  // detect peaks in fridge temperature to tune overshoot estimators
  constexpr auto INFO_NONE = 0;
  constexpr auto INFO_POSITIVE_PEAK = 1;
  constexpr auto INFO_POSITIVE_DRIFT = 2;
  constexpr auto INFO_NEGATIVE_PEAK = 3;
  constexpr auto INFO_NEGATIVE_DRIFT = 4;

  uint8_t detected = INFO_NONE;
  temperature peak, estimate, error, oldEstimator, newEstimator;

  if (_doPosPeakDetect && !stateIsHeating()) {
    peak = fridgeControl()->detectPosPeak();
    estimate = _cv.posPeakEstimate;
    error = peak - estimate;
    oldEstimator = _cs.heatEstimator;
    if (!isnan(peak)) {
      // positive peak detected
      if (error > _cc.heatingTargetUpper) {
        // Peak temperature was higher than the estimate.
        // Overshoot was higher than expected
        // Increase estimator to increase the estimated overshoot
        increaseEstimator(&(_cs.heatEstimator), error);
      }
      if (error < _cc.heatingTargetLower) {
        // Peak temperature was lower than the estimate.
        // Overshoot was lower than expected
        // Decrease estimator to decrease the estimated overshoot
        decreaseEstimator(&(_cs.heatEstimator), error);
      }
      detected = INFO_POSITIVE_PEAK;
    } else if (timeSinceHeating() > _minTimes.HEAT_PEAK_DETECT_TIME) {
      if (fridgeControl()->readFastFiltered() <
          (_cv.posPeakEstimate + _cc.heatingTargetLower)) {
        // Idle period almost reaches maximum allowed time for peak detection
        // This is the heat, then drift up too slow (but in the right
        // direction). estimator is too high
        peak = fridgeControl()->readFastFiltered();
        // There is no peak, so no error either: use the largest correction
        decreaseEstimator(&(_cs.heatEstimator), INFINITY);
        detected = INFO_POSITIVE_DRIFT;
      } else {
        // maximum time for peak estimation reached
        _doPosPeakDetect = false;
      }
    }
    if (detected != INFO_NONE) {
      newEstimator = _cs.heatEstimator;
      _cv.posPeak = peak;
      _doPosPeakDetect = false;
    }
  } else if (_doNegPeakDetect && !stateIsCooling()) {
    peak = fridgeControl()->detectNegPeak();
    estimate = _cv.negPeakEstimate;
    error = peak - estimate;
    oldEstimator = _cs.coolEstimator;
    if (!isnan(peak)) {
      // negative peak detected
      if (error < _cc.coolingTargetLower) {
        // Peak temperature was lower than the estimate.
        // Overshoot was higher than expected
        // Increase estimator to increase the estimated overshoot
        increaseEstimator(&(_cs.coolEstimator), error);
      }
      if (error > _cc.coolingTargetUpper) {
        // Peak temperature was higher than the estimate.
        // Overshoot was lower than expected
        // Decrease estimator to decrease the estimated overshoot
        decreaseEstimator(&(_cs.coolEstimator), error);
      }
      detected = INFO_NEGATIVE_PEAK;
    } else if (timeSinceCooling() > _minTimes.COOL_PEAK_DETECT_TIME) {
      if (fridgeControl()->readFastFiltered() >
          (_cv.negPeakEstimate + _cc.coolingTargetUpper)) {
        // Idle period almost reaches maximum allowed time for peak detection
        // This is the cooling, then drift down too slow (but in the right
        // direction). estimator is too high
        peak = fridgeControl()->readFastFiltered();
        // There is no peak, so no error either: use the largest correction
        decreaseEstimator(&(_cs.coolEstimator), INFINITY);
        detected = INFO_NEGATIVE_DRIFT;
      } else {
        // maximum time for peak estimation reached
        _doNegPeakDetect = false;
      }
    }

    if (detected != INFO_NONE) {
      newEstimator = _cs.coolEstimator;
      _cv.negPeak = peak;
      _doNegPeakDetect = false;
    }
  }

  if (detected != INFO_NONE) {
    // The peak and the estimate are temperatures, the estimators are not
    switch (detected) {
      case INFO_POSITIVE_PEAK:
        Log.info(F("Positive peak detected: %F, estimated: %F. Previous heat "
                   "estimator: %F, "
                   "New heat estimator: %F." CR),
                 peak, estimate, oldEstimator, newEstimator);
        break;
      case INFO_NEGATIVE_PEAK:
        Log.info(F("Negative peak detected: %F, estimated: %F. Previous cool "
                   "estimator: %F, "
                   "New cool estimator: %F." CR),
                 peak, estimate, oldEstimator, newEstimator);
        break;
      case INFO_POSITIVE_DRIFT:
        Log.info(
            F("No peak detected. Drifting up after heating, current temp: %F, "
              "estimated peak: %F. Previous heat estimator: %F, New heat "
              "estimator: "
              "%F.." CR),
            peak, estimate, oldEstimator, newEstimator);
        break;
      case INFO_NEGATIVE_DRIFT:
        Log.info(F("No peak detected. Drifting down after cooling, current "
                   "temp: %F, "
                   "estimated peak: %F. Previous cool estimator: %F, New cool "
                   "estimator: "
                   "%F.." CR),
                 peak, estimate, oldEstimator, newEstimator);
        break;
    }
  }
}

void TempControl::increaseEstimator(temperature* estimator, temperature error) {
  // 1.2 + 3.1% of error, limit between 1.2 and 1.5
  temperature factor = 614.0 / 512 + constrain(fabs(error) / 32, 0.0, 154.0 / 512);
  *estimator = factor * *estimator;
  if (*estimator < 25.0 / 512) {
    *estimator = 25.0 / 512;  // make estimator at least 0.05
  }
  storeSettings();
}

void TempControl::decreaseEstimator(temperature* estimator, temperature error) {
  // 0.833 - 3.1% of error, limit between 0.667 and 0.833
  temperature factor = 426.0 / 512 - constrain(fabs(error) / 32, 0.0, 85.0 / 512);
  *estimator = factor * *estimator;
  storeSettings();
}

ticks_seconds_t TempControl::timeSinceCooling() {
  return ticks.secondsSince(_lastCoolTime);
}

ticks_seconds_t TempControl::timeSinceHeating() {
  return ticks.secondsSince(_lastHeatTime);
}

ticks_seconds_t TempControl::timeSinceIdle() {
  return ticks.secondsSince(_lastIdleTime);
}

void TempControl::loadDefaultSettings() {
  Log.verbose(F("BREW: Loading default settings for TempControl" CR));

  _cs.setDefaults();
  setMode(ControllerMode::off);
}

void TempControl::storeConstants() {
  Log.verbose(F("BREW: Storing contants for TempControl" CR));

  _cc.save();
}

void TempControl::loadConstants() {
  Log.verbose(F("BREW: Loading constants for TempControl" CR));

  _cc.load();
  initFilters();
}

void TempControl::storeSettings() {
  Log.verbose(F("BREW: Saving settings for TempControl" CR));

  _cs.save();
  _storedBeerSetting = _cs.beerSetting;
}

void TempControl::loadSettings() {
  Log.verbose(F("BREW: Loading settings for TempControl" CR));

  _cs.load();
  _storedBeerSetting = _cs.beerSetting;
  setMode(_cs.mode, true);  // force the mode update
}

void TempControl::loadDefaultConstants() {
  Log.verbose(F("BREW: Loading default constants for TempControl" CR));

  _cc.setDefaults();
  initFilters();
}

void TempControl::initFilters() {
  Log.verbose(F("BREW: Initializing filters for TempControl" CR));

  _fridgeSensor->setFastFilterCoefficients(_cc.fridgeFastFilter);
  _fridgeSensor->setSlowFilterCoefficients(_cc.fridgeSlowFilter);
  _fridgeSensor->setSlopeFilterCoefficients(_cc.fridgeSlopeFilter);

  // A beer probe without a fridge probe also drives the fridge side of the PID,
  // which needs the lighter fridge filters (the slow beer slope filter makes
  // that loop hunt).
  if (isDefaultFridgeSensor()) {
    _beerSensor->setFastFilterCoefficients(_cc.fridgeFastFilter);
    _beerSensor->setSlowFilterCoefficients(_cc.fridgeSlowFilter);
    _beerSensor->setSlopeFilterCoefficients(_cc.fridgeSlopeFilter);
    return;
  }

  _beerSensor->setFastFilterCoefficients(_cc.beerFastFilter);
  _beerSensor->setSlowFilterCoefficients(_cc.beerSlowFilter);
  _beerSensor->setSlopeFilterCoefficients(_cc.beerSlopeFilter);
}

void TempControl::setMode(char newMode, bool force) {
  Log.verbose(F("BREW: Setting new mode %c for TempControl" CR), newMode);

  if (newMode != _cs.mode || _state == ControllerState::WAITING_TO_HEAT ||
      _state == ControllerState::WAITING_TO_COOL ||
      _state == ControllerState::WAITING_FOR_PEAK_DETECT) {
    _state = ControllerState::IDLE;
    force = true;
  }
  if (force) {
    _cs.mode = newMode;
    if (newMode == ControllerMode::off) {
      _cs.beerSetting = NAN;
      _cs.fridgeSetting = NAN;
    }
    storeSettings();
  }
}

temperature TempControl::getBeerTemp() {
  if (_beerSensor->isConnected()) {
    return _beerSensor->readFastFiltered();
  } else {
    return NAN;
  }
}

temperature TempControl::getBeerSetting() { return _cs.beerSetting; }

temperature TempControl::getFridgeTemp() {
  if (_fridgeSensor->isConnected()) {
    return _fridgeSensor->readFastFiltered();
  } else {
    return NAN;
  }
}

temperature TempControl::getFridgeSetting() { return _cs.fridgeSetting; }

void TempControl::setBeerTemp(temperature newTemp) {
  temperature oldBeerSetting = _cs.beerSetting;
  _cs.beerSetting = newTemp;
  if (isnan(oldBeerSetting) || fabs(oldBeerSetting - newTemp) >
                                   0.5) {  // more than half degree C
                                           // difference with old setting
    reset();                               // reset controller
  }
  updatePID();
  updateState();
  if (/*cs.mode != Modes::beerProfile ||*/
      isnan(_storedBeerSetting) || fabs(_storedBeerSetting - newTemp) > 0.25) {
    // more than 1/4 degree C difference with EEPROM
    // Do not store settings every time in profile mode, because EEPROM has
    // limited number of write cycles. A temperature ramp would cause a lot of
    // writes If Raspberry Pi is connected, it will update the settings anyway.
    // This is just a safety feature.

    storeSettings();
  }
}

void TempControl::setFridgeTemp(temperature newTemp) {
  _cs.fridgeSetting = newTemp;
  reset();  // reset peak detection and PID
  updatePID();
  updateState();
  storeSettings();
}

bool TempControl::stateIsCooling() {
  return (_state == ControllerState::COOLING ||
          _state == ControllerState::COOLING_MIN_TIME);
}

bool TempControl::stateIsHeating() {
  return (_state == ControllerState::HEATING ||
          _state == ControllerState::HEATING_MIN_TIME);
}

// EOF
