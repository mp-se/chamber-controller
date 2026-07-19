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
#include <ArduinoJson.h>

#include <espframework.hpp>
#include <main.hpp>
#include <pidconfig.hpp>

PidConfig::PidConfig(String baseMDNS, String fileName)
    : BaseConfig(baseMDNS, fileName) {}

void PidConfig::createJson(JsonObject& doc) const {
  // Call base class functions
  createJsonBase(doc);
  createJsonWifi(doc);
  // createJsonOta(doc);
  createJsonPush(doc);

  // Handle project specific config
  doc[PARAM_FRIDGE_SENSOR_ID] = getFridgeSensorId();
  doc[PARAM_BEER_SENSOR_ID] = getBeerSensorId();
  doc[PARAM_BEER_BLE_SENSOR_ID] = getBeerBleSensorId();
  doc[PARAM_FRIDGE_SENSOR_OFFSET] =
      serialized(String(getFridgeSensorOffset(), DECIMALS_TEMP));
  doc[PARAM_BEER_SENSOR_OFFSET] =
      serialized(String(getBeerSensorOffset(), DECIMALS_TEMP));
  doc[PARAM_CONTROLLER_MODE] = String(getControllerMode());
  doc[PARAM_TARGET_TEMPERATURE] =
      serialized(String(getTargetTemperature(), DECIMALS_TEMP));
  doc[PARAM_ENABLE_COOLING] = isCoolingEnabled();
  doc[PARAM_ENABLE_HEATING] = isHeatingEnabled();
  doc[PARAM_ENABLE_FAN] = isFanEnabled();
  doc[PARAM_INVERT_PINS] = isPinsInverted();
  doc[PARAM_RESTART_INTERVAL] = getRestartInterval();
  // doc[PARAM_BLE_ENABLED] = isBlePushEnabled(); // This variable is the same
  // as ble_push and will be removed in future
  doc[PARAM_BLE_PUSH_ENABLED] = isBlePushEnabled();
  doc[PARAM_BLE_SCAN_ENABLED] = isBleScanEnabled();
  doc[PARAM_BLE_SENSOR_VALID_TIME] = getBleSensorValidTime();
  doc[PARAM_REMOTE_CONTROL_ACTIVE] = getRemoteControlActive();
  doc[PARAM_REMOTE_PREVIOUS_BLE_SENSOR_ID] = getRemotePreviousBleSensorId();
  doc[PARAM_REMOTE_PREVIOUS_MODE] = String(getRemotePreviousMode());
  doc[PARAM_REMOTE_PREVIOUS_TARGET_TEMP] =
      serialized(String(getRemotePreviousTargetTemp(), DECIMALS_TEMP));
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
  if (!doc[PARAM_BEER_BLE_SENSOR_ID].isNull())
    setBeerBleSensorId(doc[PARAM_BEER_BLE_SENSOR_ID]);
  if (!doc[PARAM_FRIDGE_SENSOR_OFFSET].isNull())
    setFridgeSensorOffset(doc[PARAM_FRIDGE_SENSOR_OFFSET].as<float>());
  if (!doc[PARAM_BEER_SENSOR_OFFSET].isNull())
    setBeerSensorOffset(doc[PARAM_BEER_SENSOR_OFFSET].as<float>());
  if (!doc[PARAM_TARGET_TEMPERATURE].isNull())
    setTargetTemperature(doc[PARAM_TARGET_TEMPERATURE].as<float>());
  if (!doc[PARAM_ENABLE_COOLING].isNull())
    setCoolingEnabled(doc[PARAM_ENABLE_COOLING].as<bool>());
  if (!doc[PARAM_ENABLE_HEATING].isNull())
    setHeatingEnabled(doc[PARAM_ENABLE_HEATING].as<bool>());
  if (!doc[PARAM_ENABLE_FAN].isNull())
    setFanEnabled(doc[PARAM_ENABLE_FAN].as<bool>());
  if (!doc[PARAM_INVERT_PINS].isNull())
    setPinsInverted(doc[PARAM_INVERT_PINS].as<bool>());
  if (!doc[PARAM_RESTART_INTERVAL].isNull())
    setRestartInterval(doc[PARAM_RESTART_INTERVAL].as<int>());
  if (!doc[PARAM_BLE_ENABLED]
           .isNull())  // This variable is the same as ble_push and will be
                       // removed in future
    setBlePushEnabled(
        doc[PARAM_BLE_ENABLED]
            .as<bool>());  // This variable is the same as ble_push and will be
                           // removed in future
  if (!doc[PARAM_BLE_PUSH_ENABLED].isNull())
    setBlePushEnabled(doc[PARAM_BLE_PUSH_ENABLED].as<bool>());
  if (!doc[PARAM_BLE_SCAN_ENABLED].isNull())
    setBleScanEnabled(doc[PARAM_BLE_SCAN_ENABLED].as<bool>());
  if (!doc[PARAM_BLE_SENSOR_VALID_TIME].isNull())
    setBleSensorValidTime(doc[PARAM_BLE_SENSOR_VALID_TIME].as<int>());
  if (!doc[PARAM_REMOTE_CONTROL_ACTIVE].isNull())
    setRemoteControlActive(doc[PARAM_REMOTE_CONTROL_ACTIVE].as<bool>());
  if (!doc[PARAM_REMOTE_PREVIOUS_BLE_SENSOR_ID].isNull())
    setRemotePreviousBleSensorId(doc[PARAM_REMOTE_PREVIOUS_BLE_SENSOR_ID]);
  if (!doc[PARAM_REMOTE_PREVIOUS_MODE].isNull()) {
    String s = doc[PARAM_REMOTE_PREVIOUS_MODE];
    setRemotePreviousMode(s.charAt(0));
  }
  if (!doc[PARAM_REMOTE_PREVIOUS_TARGET_TEMP].isNull())
    setRemotePreviousTargetTemp(
        doc[PARAM_REMOTE_PREVIOUS_TARGET_TEMP].as<float>());
}

// EOF
