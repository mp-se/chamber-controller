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
#ifndef SRC_PIDWEBSERVER_HPP_
#define SRC_PIDWEBSERVER_HPP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <basewebserver.hpp>
#include <list>
#include <brewgraph.hpp>

class PidWebServer : public BaseWebServer {
 private:
  BrewGraphPush* _push;
  volatile bool _sensorScanTask = false;
  volatile bool _tempControllerInitTask = false;
  volatile bool _hardwareScanTask = false;
  std::list<String> _sensors;
  String _hardwareScanData;

  void setupWebHandlers();
  void webHandleStatus(AsyncWebServerRequest* request);
  void webHandleFeature(AsyncWebServerRequest* request);
  void webHandleTemps(AsyncWebServerRequest* request);
  void webHandleConfigRead(AsyncWebServerRequest* request);
  void webHandleConfigWrite(AsyncWebServerRequest* request, JsonVariant& json);

  void webHandleMode(AsyncWebServerRequest* request, JsonVariant& json);
  void webHandleListSensor(AsyncWebServerRequest* request);
  void webHandleListSensorStatus(AsyncWebServerRequest* request);
  void webHandleHardwareScan(AsyncWebServerRequest* request);
  void webHandleHardwareScanStatus(AsyncWebServerRequest* request);
  void runHardwareScan();

  void webHandleControlConstants(AsyncWebServerRequest* request);
  void webHandleControlSettings(AsyncWebServerRequest* request);
  void webHandleControlVariables(AsyncWebServerRequest* request);
  void webHandleMinTimes(AsyncWebServerRequest* request);

 public:
  explicit PidWebServer(WebConfigInterface* config, BrewGraphPush* push);

  void loop();
};

#endif  // SRC_PIDWEBSERVER_HPP_

// EOF
