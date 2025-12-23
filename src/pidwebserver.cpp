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
#include <DallasTemperature.h>
#include <LittleFS.h>
#include <OneWire.h>

#include <NumberFormats.hpp>
#include <TempControl.hpp>
#include <espframework.hpp>
#include <log.hpp>
#include <main.hpp>
#include <measurement.hpp>
#include <pidconfig.hpp>
#include <pidpush.hpp>
#include <pidwebserver.hpp>
#include <uptime.hpp>
#include <utils.hpp>

// These are parameters that the example ui app uses. Part of the status
// response.
constexpr auto PARAM_PLATFORM = "platform";
constexpr auto PARAM_BOARD = "board";
constexpr auto PARAM_TOTAL_HEAP = "total_heap";
constexpr auto PARAM_FREE_HEAP = "free_heap";
constexpr auto PARAM_IP = "ip";
constexpr auto PARAM_WIFI_SETUP = "wifi_setup";
constexpr auto PARAM_APP_VER = "app_ver";
constexpr auto PARAM_APP_BUILD = "app_build";
constexpr auto PARAM_UPTIME_SECONDS = "uptime_seconds";
constexpr auto PARAM_UPTIME_MINUTES = "uptime_minutes";
constexpr auto PARAM_UPTIME_HOURS = "uptime_hours";
constexpr auto PARAM_UPTIME_DAYS = "uptime_days";
constexpr auto PARAM_FIRMWARE_FILE = "firmware_file";
constexpr auto PARAM_FEATURE_BLE_SUPPORTED = "ble";
constexpr auto PARAM_FEATURE_BLE_SENSOR = "ble_sensor";

// Pid related params
constexpr auto PARAM_PID_MODE = "pid_mode";
constexpr auto PARAM_PID_STATE = "pid_state";
constexpr auto PARAM_PID_STATE_STRING = "pid_state_string";
constexpr auto PARAM_PID_BEER_TEMP = "pid_beer_temp";
constexpr auto PARAM_PID_BEER_TEMP_CONNECTED = "pid_beer_temp_connected";
constexpr auto PARAM_PID_FRIDGE_TEMP = "pid_fridge_temp";
constexpr auto PARAM_PID_FRIDGE_TEMP_CONNECTED = "pid_fridge_temp_connected";
constexpr auto PARAM_PID_BEER_TARGET_TEMP = "pid_beer_target_temp";
constexpr auto PARAM_PID_FRIDGE_TARGET_TEMP = "pid_fridge_target_temp";
constexpr auto PARAM_PID_TEMP_FORMAT = "pid_temp_format";
constexpr auto PARAM_PID_COOLING_ACTUATOR_ACTIVE =
    "pid_cooling_actuator_active";
constexpr auto PARAM_PID_HEATING_ACTUATOR_ACTIVE =
    "pid_heating_actuator_active";
constexpr auto PARAM_PID_WAIT_TIME = "pid_wait_time";
constexpr auto PARAM_PID_TIME_SINCE_COOLING = "pid_time_since_cooling";
constexpr auto PARAM_PID_TIME_SINCE_HEATING = "pid_time_since_heating";
constexpr auto PARAM_PID_TIME_SINCE_IDLE = "pid_time_since_idle";

constexpr auto PARAM_NEW_MODE = "new_mode";
constexpr auto PARAM_NEW_TEMPERATURE = "new_temperature";

constexpr auto PARAM_SENSORS = "sensors";

// Ble sensors
constexpr auto PARAM_TEMPERATURE_DEVICE = "temperature_device";
constexpr auto PARAM_DEVICE = "device";
constexpr auto PARAM_NAME = "name";
constexpr auto PARAM_TYPE = "type";
constexpr auto PARAM_SOURCE = "source";
constexpr auto PARAM_TEMP = "temp";
constexpr auto PARAM_UPDATE_TIME = "update_time";

extern OneWire oneWire;

PidWebServer::PidWebServer(WebConfigInterface *config, PidPush *push)
    : BaseWebServer(config) {
  _push = push;
}

void PidWebServer::setupWebHandlers() {
  Log.notice(F("WEB : Setting up web handlers." CR));
  BaseWebServer::setupWebHandlers();

  MDNS.addService("chamberctl", "tcp", 80);
  MDNS.addServiceTxt("chamberctl", "tcp", "ver", CFG_APPVER);
  MDNS.addServiceTxt("chamberctl", "tcp", "app", CFG_APPNAME);
  MDNS.addServiceTxt("chamberctl", "tcp", "id", _webConfig->getID());

  _server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleStatus(request);
  });
  _server->on("/api/temps", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleTemps(request);
  });
  _server->on("/api/feature", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleFeature(request);
  });

  AsyncCallbackJsonWebHandler *handler;
  _server->on("/api/config", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleConfigRead(request);
  });
  handler = new AsyncCallbackJsonWebHandler(
      "/api/config", [this](AsyncWebServerRequest *request, JsonVariant &json) {
        this->webHandleConfigWrite(request, json);
      });
  _server->addHandler(handler);
  handler = new AsyncCallbackJsonWebHandler(
      "/api/mode", [this](AsyncWebServerRequest *request, JsonVariant &json) {
        this->webHandleMode(request, json);
      });
  _server->addHandler(handler);
  _server->on("/api/sensor/status", HTTP_GET,
              [this](AsyncWebServerRequest *request) {
                this->webHandleListSensorStatus(request);
              });
  _server->on("/api/sensor", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleListSensor(request);
  });
  _server->on("/api/pid/cc", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleControlConstants(request);
  });
  _server->on("/api/pid/cs", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleControlSettings(request);
  });
  _server->on("/api/pid/cv", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleControlVariables(request);
  });
  _server->on("/api/pid/mt", HTTP_GET, [this](AsyncWebServerRequest *request) {
    this->webHandleMinTimes(request);
  });
}

void PidWebServer::webHandleFeature(AsyncWebServerRequest *request) {
  Log.notice(F("WEB : webServer callback for /api/feature(get)." CR));

  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();

  obj[PARAM_PLATFORM] = platform;
#if defined(BOARD)
  obj[PARAM_BOARD] = BOARD;
#else
  obj[PARAM_BOARD] = "UNDEFINED";
#endif
  obj[PARAM_APP_VER] = CFG_APPVER;
  obj[PARAM_APP_BUILD] = CFG_GITREV;
  obj[PARAM_FIRMWARE_FILE] = CFG_FILENAMEBIN;
#if defined(ENABLE_BLE)
  obj[PARAM_FEATURE_BLE_SUPPORTED] = true;
#else
  obj[PARAM_FEATURE_BLE_SUPPORTED] = false;
#endif

#if defined(ENABLE_BLE) && defined(ENABLE_BLE_SENSOR)
  obj[PARAM_FEATURE_BLE_SENSOR] = true;
#else
  obj[PARAM_FEATURE_BLE_SENSOR] = false;
#endif

  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleConfigRead(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/config(read)." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  _webConfig->createJson(obj);
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleConfigWrite(AsyncWebServerRequest *request,
                                        JsonVariant &json) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/config(write)." CR));
  JsonObject obj = json.as<JsonObject>();
  _webConfig->parseJson(obj);
  _webConfig->saveFile();

  // If the hardware setup is changed we need to reinit the temperature
  // controller.
  if (!obj[PARAM_FRIDGE_SENSOR_ID].isNull() ||
      !obj[PARAM_BEER_SENSOR_ID].isNull() ||
      !obj[PARAM_ENABLE_COOLING].isNull() ||
      !obj[PARAM_ENABLE_HEATING].isNull() || !obj[PARAM_TEMP_FORMAT].isNull()) {
    // TODO: Check probe delta
    _tempControllerInitTask = true;
  }

  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  obj = response->getRoot().as<JsonObject>();
  obj[PARAM_SUCCESS] = true;
  obj[PARAM_MESSAGE] = "Configuration updated";
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleStatus(AsyncWebServerRequest *request) {
  // Log.notice(F("WEB : webServer callback for /api/status." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();

  // Generic params
  obj[PARAM_ID] = _webConfig->getID();
  obj[PARAM_MDNS] = _webConfig->getMDNS();
  obj[PARAM_RSSI] = WiFi.RSSI();
  obj[PARAM_SSID] = WiFi.SSID();
  obj[PARAM_TOTAL_HEAP] = ESP.getHeapSize();
  obj[PARAM_FREE_HEAP] = ESP.getFreeHeap();
  obj[PARAM_IP] = WiFi.localIP().toString();
  obj[PARAM_WIFI_SETUP] = _wifiSetup;
  obj[PARAM_UPTIME_SECONDS] = myUptime.getSeconds();
  obj[PARAM_UPTIME_MINUTES] = myUptime.getMinutes();
  obj[PARAM_UPTIME_HOURS] = myUptime.getHours();
  obj[PARAM_UPTIME_DAYS] = myUptime.getDays();

  // Pid controller
  if (runMode == RunMode::pidMode) {
    obj[PARAM_PID_MODE] = String(tempControl.getMode());
    obj[PARAM_PID_STATE] = tempControl.getState();
    obj[PARAM_PID_STATE_STRING] = tempControl.getStateAsString();
    obj[PARAM_PID_BEER_TEMP] = tempControl.getBeerTemperature();
    obj[PARAM_PID_BEER_TEMP_CONNECTED] =
        tempControl.getBeerSensor()->isConnected();
    obj[PARAM_PID_FRIDGE_TEMP] =
        serialized(String(tempControl.getFridgeTemperature(), DECIMALS_TEMP));
    obj[PARAM_PID_FRIDGE_TEMP_CONNECTED] =
        tempControl.getFridgeSensor()->isConnected();
    obj[PARAM_PID_BEER_TARGET_TEMP] = serialized(
        String(tempControl.getBeerTemperatureSetting(), DECIMALS_TEMP));
    obj[PARAM_PID_FRIDGE_TARGET_TEMP] = serialized(
        String(tempControl.getFridgeTemperatureSetting(), DECIMALS_TEMP));
    obj[PARAM_PID_TEMP_FORMAT] =
        String(tempControl.getControlConstants().tempFormat);
    obj[PARAM_PID_COOLING_ACTUATOR_ACTIVE] =
        tempControl.getCoolingActuator()->isActive();
    obj[PARAM_PID_HEATING_ACTUATOR_ACTIVE] =
        tempControl.getHeatingActuator()->isActive();
    obj[PARAM_PID_WAIT_TIME] = tempControl.getWaitTime();
    obj[PARAM_PID_TIME_SINCE_COOLING] = tempControl.timeSinceCooling();
    obj[PARAM_PID_TIME_SINCE_HEATING] = tempControl.timeSinceHeating();
    obj[PARAM_PID_TIME_SINCE_IDLE] = tempControl.timeSinceIdle();
  }

  JsonArray temperatureDevices = obj[PARAM_TEMPERATURE_DEVICE].to<JsonArray>();
  int tempIdx = 0;

#if defined(ENABLE_BLE) && defined(ENABLE_BLE_SENSOR)
  for (int i = 0; i < myMeasurementList.size(); i++) {
    MeasurementEntry *entry = myMeasurementList.getMeasurementEntry(i);

    switch (entry->getType()) {
      case MeasurementType::Gravitymon: {
        Log.notice("WEB: Processing Gravitymon data %d." CR, i);
        const GravityData *gd = entry->getGravityData();

        temperatureDevices[tempIdx][PARAM_NAME] = gd->getName();
        temperatureDevices[tempIdx][PARAM_DEVICE] = gd->getId();
        temperatureDevices[tempIdx][PARAM_TEMP] = gd->getTempC();
        temperatureDevices[tempIdx][PARAM_UPDATE_TIME] = entry->getUpdateAge();
        temperatureDevices[tempIdx][PARAM_SOURCE] = gd->getSourceAsString();
        temperatureDevices[tempIdx][PARAM_TYPE] = gd->getTypeAsString();
        tempIdx++;
      } break;

      case MeasurementType::Tilt:
      case MeasurementType::TiltPro: {
        Log.notice("WEB: Processing Tilt data %d." CR, i);
        const TiltData *td = entry->getTiltData();

        temperatureDevices[tempIdx][PARAM_NAME] = td->getTiltColor();
        temperatureDevices[tempIdx][PARAM_DEVICE] = td->getId();
        temperatureDevices[tempIdx][PARAM_TEMP] = td->getTempC();
        temperatureDevices[tempIdx][PARAM_UPDATE_TIME] = entry->getUpdateAge();
        temperatureDevices[tempIdx][PARAM_SOURCE] = td->getSourceAsString();
        temperatureDevices[tempIdx][PARAM_TYPE] = td->getTypeAsString();
        tempIdx++;
      } break;

      case MeasurementType::Rapt: {
        Log.notice("WEB: Processing Rapt data %d." CR, i);
        const RaptData *rd = entry->getRaptData();

        temperatureDevices[tempIdx][PARAM_NAME] = rd->getId();
        temperatureDevices[tempIdx][PARAM_DEVICE] = rd->getId();
        temperatureDevices[tempIdx][PARAM_TEMP] = rd->getTempC();
        temperatureDevices[tempIdx][PARAM_UPDATE_TIME] = entry->getUpdateAge();
        temperatureDevices[tempIdx][PARAM_SOURCE] = rd->getSourceAsString();
        temperatureDevices[tempIdx][PARAM_TYPE] = rd->getTypeAsString();
        tempIdx++;
      } break;
    }
  }
#endif  // ENABLE_BLE && ENABLE_BLE_SENSOR

  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleTemps(AsyncWebServerRequest *request) {
  // Log.notice(F("WEB : webServer callback for /api/temps." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();

  if (runMode == RunMode::pidMode) {
    obj[PARAM_PID_BEER_TEMP] = tempControl.getBeerTemperature();
    obj[PARAM_PID_FRIDGE_TEMP] = tempControl.getFridgeTemperature();
    obj[PARAM_PID_BEER_TARGET_TEMP] = tempControl.getBeerTemperatureSetting();
    obj[PARAM_PID_FRIDGE_TARGET_TEMP] =
        tempControl.getFridgeTemperatureSetting();
    obj[PARAM_PID_TEMP_FORMAT] =
        String(tempControl.getControlConstants().tempFormat);
  }

  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleMode(AsyncWebServerRequest *request,
                                 JsonVariant &json) {
  if (!isAuthenticated(request) || runMode != RunMode::pidMode) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/mode." CR));
  JsonObject obj = json.as<JsonObject>();
  bool success = false;
  String message = "Mode updated";

  if (!obj[PARAM_NEW_MODE].isNull() && !obj[PARAM_NEW_TEMPERATURE].isNull()) {
    char newMode = obj[PARAM_NEW_MODE].as<String>().charAt(0);
    float newTemp = obj[PARAM_NEW_TEMPERATURE].as<float>();

    Log.notice(F("WEB : Target mode %c, temp %F." CR), newMode, newTemp);

    switch (newMode) {
      case ControllerMode::beerConstant:
        if (myConfig.isBeerSensorEnabled() &&
            (myConfig.isCoolingEnabled() || myConfig.isHeatingEnabled())) {
          setNewControllerMode(ControllerMode::beerConstant, newTemp);
          success = true;
        } else {
          success = false;
          message = "Lack beer sensor or cooling/heating actuator";
        }
        break;

      case ControllerMode::fridgeConstant:
        if (myConfig.isFridgeSensorEnabled() &&
            (myConfig.isCoolingEnabled() || myConfig.isHeatingEnabled())) {
          setNewControllerMode(ControllerMode::fridgeConstant, newTemp);
          success = true;
        } else {
          success = false;
          message = "Lack chamber sensor or cooling/heating actuator";
        }
        break;

      case ControllerMode::off:
        setNewControllerMode(ControllerMode::off, newTemp);
        success = true;
        break;
    }
  }

  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  obj = response->getRoot().as<JsonObject>();
  obj[PARAM_SUCCESS] = success;
  obj[PARAM_PID_MODE] = String(tempControl.getMode());
  obj[PARAM_PID_BEER_TARGET_TEMP] = tempControl.getBeerTemperatureSetting();
  obj[PARAM_PID_FRIDGE_TARGET_TEMP] = tempControl.getFridgeTemperatureSetting();
  obj[PARAM_MESSAGE] = message;
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleControlConstants(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/pid/cc." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  if (runMode == RunMode::pidMode)
    tempControl.getControlConstants().toJsonReadable(obj);
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleControlSettings(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/pid/cs." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  if (runMode == RunMode::pidMode)
    tempControl.getControlSettings().toJsonReadable(obj);
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleControlVariables(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/pid/cv." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  if (runMode == RunMode::pidMode)
    tempControl.getControlVariables().toJsonReadable(obj);
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleMinTimes(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/pid/mt." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  if (runMode == RunMode::pidMode)
    tempControl.getMinTimes().toJsonReadable(obj);
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleListSensor(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/sensor." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();

  obj[PARAM_SUCCESS] = true;
  obj[PARAM_MESSAGE] = "Scheduled sensor scanning";
  _sensorScanTask = true;
  response->setLength();
  request->send(response);
}

void PidWebServer::webHandleListSensorStatus(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/sensor/status." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();

  obj[PARAM_STATUS] = static_cast<bool>(_sensorScanTask);

  if (_sensorScanTask) {
    obj[PARAM_SUCCESS] = false;
    obj[PARAM_MESSAGE] = "Sensor scanning is running";

  } else {
    obj[PARAM_SUCCESS] = true;
    obj[PARAM_MESSAGE] = "Sensor scanning completed";

    for (String sensor : _sensors) {
      Log.notice(F("WEB : Adding %s." CR), sensor.c_str());
      obj[PARAM_SENSORS].add(sensor);
    }
  }

  response->setLength();
  request->send(response);
}

void PidWebServer::loop() {
  BaseWebServer::loop();

  if (_tempControllerInitTask) {
    Log.notice(F("WEB : Temp controller settings changed, reinitializing." CR));
    configureTempControl();
    _tempControllerInitTask = false;
  }

  if (_sensorScanTask) {
    Log.notice(F("WEB : Scanning for temperature sensors." CR));

    _sensors.clear();

    DallasTemperature tempSensor(&oneWire);
    tempSensor.begin();

    for (int i = 0; i < tempSensor.getDS18Count(); i++) {
      char buf[17];
      DeviceAddress da;
      tempSensor.getAddress(&da[0], i);
      printBytes(da, 8, buf);
      Log.info(F("WEB: Found DS18B20 sensor at %s" CR), buf);
      _sensors.push_back(String(buf));
    }

    _sensorScanTask = false;
  }
}
// EOF
