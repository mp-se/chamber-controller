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
#ifndef SRC_PIDWEBSERVER_HPP_
#define SRC_PIDWEBSERVER_HPP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <basewebserver.hpp>
#include <list>
#include <pidpush.hpp>

class PidWebServer : public BaseWebServer {
 private:
  PidPush *_push;
  volatile bool _sensorScanTask = false;
  volatile bool _tempControllerInitTask = false;
  std::list<String> _sensors;

  void setupWebHandlers();
  void webHandleStatus(AsyncWebServerRequest *request);
  void webHandleTemps(AsyncWebServerRequest *request);
  void webHandleConfigRead(AsyncWebServerRequest *request);
  void webHandleConfigWrite(AsyncWebServerRequest *request, JsonVariant &json);

  void webHandleMode(AsyncWebServerRequest *request, JsonVariant &json);
  void webHandleListSensor(AsyncWebServerRequest *request);
  void webHandleListSensorStatus(AsyncWebServerRequest *request);

  void webHandleControlConstants(AsyncWebServerRequest *request);
  void webHandleControlSettings(AsyncWebServerRequest *request);
  void webHandleControlVariables(AsyncWebServerRequest *request);
  void webHandleMinTimes(AsyncWebServerRequest *request);

 public:
  explicit PidWebServer(WebConfig *config, PidPush *push);

  void loop();
};

#endif  // SRC_PIDWEBSERVER_HPP_

// EOF
