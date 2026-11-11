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
#include <TempControl.hpp>
#include <log.hpp>
#include <main.hpp>
#include <pidconfig.hpp>
#include <remotecontrol.hpp>

const char* remoteModeBlockedReason(char mode) {
  bool actuator = myConfig.isCoolingEnabled() || myConfig.isHeatingEnabled();

  switch (mode) {
    case ControllerMode::beerConstant:
      if (myConfig.isBleBeerSensorWithoutChamberSensor())
        return "A BLE beer sensor needs a chamber sensor";
      if (!((myConfig.isBeerSensorEnabled() ||
             myConfig.isBeerBleSensorEnabled()) &&
            actuator))
        return "Lack beer sensor or cooling/heating actuator";
      return nullptr;

    case ControllerMode::fridgeConstant:
      if (!(myConfig.isFridgeSensorEnabled() && actuator))
        return "Lack chamber sensor or cooling/heating actuator";
      return nullptr;

    default:
      return nullptr;
  }
}

RemoteResult applyRemoteCommand(char newMode, float newTemp,
                                const String& bleSensorId) {
  RemoteResult result;
  String newBleSensorId = bleSensorId;

  if (myConfig.getRemoteControlActive()) {
    if (newMode == 'r') {  // If mode is = R then we restore the saved setttings
                           // and leave remote mode

      Log.notice(
          F("REMO: Restoring previous settings and leaving remote mode." CR));

      // Use saved settings
      newMode = myConfig.getRemotePreviousMode();
      newTemp = myConfig.getRemotePreviousTargetTemp();
      if (myConfig.isBleScanEnabled()) {
        // Only restore ble sensor if beer ble sensor is enabled, otherwise
        // ignore since it might cause issues if user has disabled beer ble
        // sensor after activating remote mode
        newBleSensorId = myConfig.getRemotePreviousBleSensorId();
      }
      // Inactivate remote mode
      myConfig.setRemotePreviousMode(ControllerMode::off);
      myConfig.setRemotePreviousTargetTemp(10.0);
      myConfig.setRemotePreviousBleSensorId("");
      myConfig.setRemoteControlActive(false);
      myConfig.saveFile();
    }

    // Continue with main loop to set the new mode and temp

  } else {
    if (newMode == 'r') {  // If mode is = R then ignore, not valid since we are
                           // not in remote mode
      return result;
    }

    if (isnan(newTemp)) {
      return result;
    }

    // If remote control is not enabled we store the settings so we can restore
    // them later.
    if (!myConfig.getRemoteControlActive()) {
      myConfig.setRemotePreviousMode(myConfig.getControllerMode());
      myConfig.setRemotePreviousTargetTemp(myConfig.getTargetTemperature());
      if (myConfig.isBleScanEnabled() && newBleSensorId.length() > 0) {
        myConfig.setRemotePreviousBleSensorId(myConfig.getBeerBleSensorId());
      }
    }

    // Activate remote mode
    myConfig.setRemoteControlActive(true);

    if (myConfig.isBleScanEnabled() &&
        newBleSensorId.length() >
            0) {  // Only restore ble sensor if beer ble sensor is enabled,
                  // otherwise ignore since it might cause issues if user has
                  // disabled beer ble sensor after activating remote mode
      myConfig.setBeerBleSensorId(newBleSensorId);
    }

    myConfig.saveFile();
  }

  Log.notice(F("REMO: Target mode %c, temp %F, sensor %s." CR), newMode,
             newTemp, newBleSensorId.c_str());

  if (isnan(newTemp)) {
    return result;
  }

  switch (newMode) {
    case ControllerMode::beerConstant:
    case ControllerMode::fridgeConstant: {
      const char* blocked = remoteModeBlockedReason(newMode);
      if (blocked) {
        result.success = false;
        result.message = blocked;
      } else {
        setNewControllerMode(newMode, newTemp);
        result.success = true;
        result.message = newMode == ControllerMode::beerConstant
                             ? "Set mode to Beer Constant"
                             : "Set mode to Fridge Constant";
      }
    } break;

    case ControllerMode::off:
      setNewControllerMode(ControllerMode::off, newTemp);
      result.success = true;
      result.message = "Turning off controller";
      break;

    default:
      return result;
  }

  result.valid = true;
  return result;
}

void applyChamberReply(const ChamberReply& reply) {
  if (!myConfig.isBrewgraphRemoteControlEnabled()) {
    Log.info(F("BREW: BrewGraph control is disabled, ignoring reply." CR));
    return;
  }

  const ControlConstants& cc = tempControl.getControlConstants();
  ChamberDecision decision = decideChamberAction(
      reply, myConfig.getRemoteControlActive(), myConfig.getControllerMode(),
      myConfig.getTargetTemperature(), cc.tempSettingMin, cc.tempSettingMax);

  switch (decision.action) {
    case ChamberAction::none:
      return;

    case ChamberAction::restore:
      Log.info(F("BREW: BrewGraph has no active batch, leaving remote mode." CR));
      applyRemoteCommand('r', NAN, "");
      return;

    case ChamberAction::applyRemote: {
      // A reply the device can not follow must not lock the local controls
      const char* blocked = remoteModeBlockedReason(decision.mode);
      if (blocked) {
        Log.warning(F("BREW: BrewGraph mode %c ignored: %s." CR), decision.mode,
                    blocked);
        return;
      }
      Log.info(F("BREW: BrewGraph sets mode %c, target %F." CR), decision.mode,
               decision.target);
      applyRemoteCommand(decision.mode, decision.target, "");
      return;
    }
  }
}

// EOF
