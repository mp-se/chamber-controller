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
#ifndef SRC_MAIN_HPP_
#define SRC_MAIN_HPP_

enum RunMode {
  pidMode = 0,
  wifiSetupMode = 1,
};
extern RunMode runMode;

void configureTempControl();
void validateTempControl();
void setNewControllerMode(char mode, float temp);

constexpr auto PARAM_FRIDGE_SENSOR_ID = "fridge_sensor_id";
constexpr auto PARAM_BEER_SENSOR_ID = "beer_sensor_id";
constexpr auto PARAM_BEER_BLE_SENSOR_ID = "beer_ble_sensor_id";
constexpr auto PARAM_CONTROLLER_MODE = "controller_mode";
constexpr auto PARAM_TARGET_TEMPERATURE = "target_temperature";
constexpr auto PARAM_ENABLE_COOLING = "enable_cooling";
constexpr auto PARAM_ENABLE_HEATING = "enable_heating";
constexpr auto PARAM_ENABLE_FAN = "enable_fan";
constexpr auto PARAM_INVERT_PINS = "invert_pins";
constexpr auto PARAM_RESTART_INTERVAL = "restart_interval";
constexpr auto PARAM_BLE_ENABLED =
    "ble_enabled";  // This variable is the same as ble_push and will be removed
                    // in future
constexpr auto PARAM_BLE_SCAN_ENABLED = "ble_scan_enabled";
constexpr auto PARAM_BLE_PUSH_ENABLED = "ble_push_enabled";
constexpr auto PARAM_BLE_SENSOR_VALID_TIME = "ble_sensor_valid_time";
constexpr auto PARAM_FRIDGE_SENSOR_OFFSET = "fridge_sensor_offset";
constexpr auto PARAM_BEER_SENSOR_OFFSET = "beer_sensor_offset";

constexpr auto PARAM_REMOTE_CONTROL_ACTIVE = "remote_control_active";
constexpr auto PARAM_REMOTE_PREVIOUS_BLE_SENSOR_ID =
    "remote_previous_ble_sensor_id";
constexpr auto PARAM_REMOTE_PREVIOUS_MODE = "remote_previous_mode";
constexpr auto PARAM_REMOTE_PREVIOUS_TARGET_TEMP =
    "remote_previous_target_temp";

#define DECIMALS_TEMP 2

#endif  // SRC_MAIN_HPP_
