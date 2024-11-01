/*
MIT License

Copyright (c) 2021-2024 Magnus

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
#include <baseconfig.hpp>
#include <basewebserver.hpp>
#include <espframework.hpp>

#if defined(ESP8266)
#undef MAX_SKETCH_SPACE
#define MAX_SKETCH_SPACE 1044464
#else
#include <HTTPUpdate.h>
#if !defined(MAX_SKETCH_SPACE)
#define MAX_SKETCH_SPACE 0x1c0000
#warning "MAX_SKETCH_SPACE is not defined, using default value of 0x1c0000"
#endif
#endif

#if defined(ESP8266)
#define INCBIN_OUTPUT_SECTION ".irom.text"
#include <incbin.hpp>
// These are used in the webhandler class and needs to be defined when using
// ESP8266. For esp32 the files are defined in the platformio.ini file
INCBIN(IndexHtml, "html/index.html");
INCBIN(AppJs, "html/app.js.gz");
INCBIN(AppCss, "html/app.css.gz");
INCBIN(FaviconIco, "html/favicon.ico.gz");
#endif

BaseWebServer::BaseWebServer(WebConfig *config) { _webConfig = config; }

bool BaseWebServer::isAuthenticated(AsyncWebServerRequest *request) {
  resetWifiPortalTimer();

  if (request->hasHeader("Authorization")) {
    String token("Bearer ");
    token += _webConfig->getID();

    if (request->getHeader("Authorization")->value() == token) {
      return true;
    }
  }

  Log.info(F("WEB : No valid authorization header found, returning error 401. "
             "Url %s" CR),
           request->url().c_str());
  AsyncWebServerResponse *response = request->beginResponse(401);
  request->send(response);
  return false;
}

void BaseWebServer::loop() {
  if (!_server) return;

#if defined(ESP8266)
  MDNS.update();
#endif

  if (_wifiSetup) {
    if (abs((int32_t)(millis() - _wifiPortalTimer)) >
        (_webConfig->getWifiPortalTimeout() * 1000)) {
      Log.notice(F("WEB : Wifi portal timeout, reboot device." CR));
      delay(500);
      ESP_RESET();
    }
  }

  if (_rebootTask) {
    if (abs((int32_t)(millis() - _rebootTimer)) > 1000) {
      Log.notice(F("WEB : Rebooting..." CR));
      delay(500);
      ESP_RESET();
    }
  }

  if (_wifiScanTask) {
    JsonDocument doc;

    doc[PARAM_STATUS] = false;
    doc[PARAM_SUCCESS] = true;
    doc[PARAM_MESSAGE] = "";

    JsonArray networks = doc[PARAM_NETWORKS].to<JsonArray>();

    Log.notice(F("WEB : Scanning for wifi networks." CR));
    int noNetwork = WiFi.scanNetworks(false, false);

    for (int i = 0; i < noNetwork; i++) {
      networks[i][PARAM_SSID] = WiFi.SSID(i);
      networks[i][PARAM_RSSI] = WiFi.RSSI(i);
      networks[i][PARAM_CHANNEL] = WiFi.channel(i);
#if defined(ESP8266)
      networks[i][PARAM_ENCRYPTION] = WiFi.encryptionType(i);
#else
      networks[i][PARAM_ENCRYPTION] = WiFi.encryptionType(i);
#endif
    }

    unsigned int ret = serializeJson(doc, _wifiScanData);
    Log.notice(F("WEB : Scan complete %d, %s." CR), ret, _wifiScanData.c_str());
    _wifiScanTask = false;
  }
}

void BaseWebServer::webHandleUploadFirmware(AsyncWebServerRequest *request,
                                            String filename, size_t index,
                                            uint8_t *data, size_t len,
                                            bool final) {
  if (!isAuthenticated(request)) {
    return;
  }

  uint32_t maxSketchSpace = MAX_SKETCH_SPACE;
  Log.verbose(F("WEB : BaseWebHandler callback for /api/firmware." CR));

  if (!index) {
    _uploadedSize = 0;
#if defined(ESP8266)
    Update.runAsync(true);
#endif
    if (!Update.begin(request->contentLength(), U_FLASH, LED_BUILTIN)) {
      _uploadReturn = 500;
      Log.error(F("WEB : Not enough space to store for this firmware (%d)." CR),
                request->contentLength());
    } else {
      _uploadReturn = 200;
      Log.notice(F("WEB : Start firmware upload, max sketch size %d kb, size "
                   "%d kb." CR),
                 maxSketchSpace / 1024, request->contentLength() / 1024);
    }
  }

  _uploadedSize += len;

  if (_uploadedSize > maxSketchSpace) {
    _uploadReturn = 500;
    Log.error(F("WEB : Firmware file is to large." CR));
  } else if (Update.write(data, len) != len) {
    _uploadReturn = 500;
    Log.notice(F("WEB : Writing firmware upload %d (%d)." CR), len,
               maxSketchSpace);
  } else {
    EspSerial.print(".");
  }

  if (final) {
    EspSerial.print("\n");
    Log.notice(F("WEB : Finished firmware upload." CR));
    request->send(200);

    if (Update.end(true)) {
      // Calling reset here will not wait for all the data to be sent, lets wait
      // a second before rebooting in loop.
      Log.notice(F("WEB : Scheduling reboot." CR));
      _rebootTimer = millis();
      _rebootTask = true;
    } else {
      Log.error(F("WEB : Failed to finish firmware flashing, error %d" CR),
                Update.getError());
      _uploadReturn = 500;
    }
  }
}

void BaseWebServer::webHandleUploadFile(AsyncWebServerRequest *request,
                                        String filename, size_t index,
                                        uint8_t *data, size_t len, bool final) {
  if (!isAuthenticated(request)) {
    return;
  }

#if defined(ESP8266)
  FSInfo info;
  LittleFS.info(info);
  uint32_t maxFileSize = info.totalBytes - info.usedBytes - 4096;
#else
  uint32_t maxFileSize = LittleFS.totalBytes() - LittleFS.usedBytes() - 4096;
#endif
  Log.verbose(
      F("WEB : BaseWebHandler callback for /api/filesystem/upload." CR));

  if (!index) {
    Log.notice(F("WEB : Start file upload, free space %d kb, size %d." CR),
               maxFileSize / 1024, request->contentLength());

    if (len > maxFileSize) {
      Log.error(F("WEB : File is to large to fit in file system." CR));
      request->send(500);
      return;
    }

    request->_tempFile = LittleFS.open("/" + filename, "w");
    _uploadReturn = 200;
  }

  if (len) {
    request->_tempFile.write(data, len);
    EspSerial.print(".");
  }

  if (final) {
    Log.notice(F("WEB : Finished file upload." CR));
    request->_tempFile.close();
    request->send(200);
  }
}

void BaseWebServer::webHandlePageNotFound(AsyncWebServerRequest *request) {
  if (_wifiSetup) {
    request->redirect("http://192.168.4.1");
    return;
  }

  if (request->method() == HTTP_OPTIONS) {
    Log.notice(F("WEB : Got OPTIONS request for %s." CR),
               request->url().c_str());
    if (_webConfig->isCorsAllowed()) {
      AsyncWebServerResponse *resp = request->beginResponse(200);
      resp->addHeader("Access-Control-Allow-Credentials", "true");
      resp->addHeader("Access-Control-Allow-Methods",
                      "GET,HEAD,OPTIONS,POST,PUT");
      resp->addHeader(
          "Access-Control-Allow-Headers",
          "Access-Control-Allow-Headers, Origin,Accept, X-Requested-With, "
          "Content-Type, Access-Control-Request-Method, "
          "Access-Control-Request-Headers, Authorization");
      request->send(resp);
      return;
    } else {
      Log.error(F("WEB : CORS is not enabled in configuration, ignoring OPTIONS request." CR));
    }
  }

  if (request->method() == HTTP_GET)
    Log.warning(F("WEB : GET on %s not recognized." CR),
                request->url().c_str());
  else if (request->method() == HTTP_POST)
    Log.warning(F("WEB : POST on %s not recognized." CR),
                request->url().c_str());
  else if (request->method() == HTTP_PUT)
    Log.warning(F("WEB : PUT on %s not recognized." CR),
                request->url().c_str());
  else if (request->method() == HTTP_DELETE)
    Log.warning(F("WEB : DELETE on %s not recognized." CR),
                request->url().c_str());
  else
    Log.warning(F("WEB : Unknown on %s not recognized." CR),
                request->url().c_str());

  request->redirect("/");
}

void BaseWebServer::webHandleAuth(AsyncWebServerRequest *request) {
  Log.notice(F("WEB : webServer callback for /api/auth." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  obj[PARAM_TOKEN] = _webConfig->getID();
  response->setLength();
  request->send(response);
}

void BaseWebServer::webHandleFileSystem(AsyncWebServerRequest *request,
                                        JsonVariant &json) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/filesystem." CR));
  JsonObject obj = json.as<JsonObject>();

  if (!obj[PARAM_COMMAND].isNull()) {
    if (obj[PARAM_COMMAND] == String("dir")) {
      Log.notice(F("WEB : File system listing requested." CR));
      AsyncJsonResponse *response = new AsyncJsonResponse(false);
      JsonObject obj = response->getRoot().as<JsonObject>();

#if defined(ESP8266)
      FSInfo info;
      LittleFS.info(info);

      obj[PARAM_TOTAL] = info.totalBytes;
      obj[PARAM_USED] = info.usedBytes;
      obj[PARAM_FREE] = info.totalBytes - info.usedBytes;

      Dir dir = LittleFS.openDir("/");
      int i = 0;
      JsonArray arr = obj[PARAM_FILES].to<JsonArray>();

      while (dir.next()) {
        arr[i][PARAM_FILE] = "/" + String(dir.fileName());
        arr[i][PARAM_SIZE] = static_cast<int>(dir.fileSize());
        arr.add(file);
        i++;
      }
#else  // ESP32
      obj[PARAM_TOTAL] = LittleFS.totalBytes();
      obj[PARAM_USED] = LittleFS.usedBytes();
      obj[PARAM_FREE] = LittleFS.totalBytes() - LittleFS.usedBytes();

      File root = LittleFS.open("/");
      File f = root.openNextFile();
      int i = 0;

      JsonArray arr = obj[PARAM_FILES].to<JsonArray>();
      while (f) {
        arr[i][PARAM_FILE] = "/" + String(f.name());
        arr[i][PARAM_SIZE] = static_cast<int>(f.size());
        f = root.openNextFile();
        i++;
      }
      f.close();
      root.close();
#endif
      response->setLength();
      request->send(response);
    } else if (obj[PARAM_COMMAND] == String("del")) {
      Log.notice(F("WEB : File system delete requested." CR));

      if (!obj[PARAM_FILE].isNull()) {
        String f = obj[PARAM_FILE];
        LittleFS.remove(f);
        request->send(200);
      } else {
        request->send(400);
      }
    } else if (obj[PARAM_COMMAND] == String("get")) {
      Log.notice(F("WEB : File system get requested." CR));
      if (!obj[PARAM_FILE].isNull()) {
        String f = obj[PARAM_FILE];

        if (LittleFS.exists(obj[PARAM_FILE].as<String>())) {
          AsyncWebServerResponse *response =
              request->beginResponse(LittleFS, f, "");
          request->send(response);
        } else {
          request->send(404);
        }
      } else {
        request->send(400);
      }
    } else {
      Log.warning(F("WEB : Unknown file system command." CR));
      request->send(400);
    }
  } else {
    Log.warning(F("WEB : Unknown file system command." CR));
    request->send(400);
  }
}

void BaseWebServer::webHandleWifiScan(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/wifi/scan." CR));
  _wifiScanTask = true;
  _wifiScanData = "";
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  obj[PARAM_SUCCESS] = true;
  obj[PARAM_MESSAGE] = "Scheduled wifi scanning";
  response->setLength();
  request->send(response);
}

void BaseWebServer::webHandleWifiScanStatus(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/wifi/scan/status." CR));

  if (_wifiScanTask || !_wifiScanData.length()) {
    AsyncJsonResponse *response = new AsyncJsonResponse(false);
    JsonObject obj = response->getRoot().as<JsonObject>();
    obj[PARAM_STATUS] = static_cast<bool>(_wifiScanTask);
    obj[PARAM_SUCCESS] = false;
    obj[PARAM_MESSAGE] =
        _wifiScanTask ? "Wifi scanning running" : "No scanning running";
    response->setLength();
    request->send(response);
  } else {
    request->send(200, "application/json", _wifiScanData);
  }
}

void BaseWebServer::webHandleRestart(AsyncWebServerRequest *request) {
  if (!isAuthenticated(request)) {
    return;
  }

  Log.notice(F("WEB : webServer callback for /api/restart." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  obj[PARAM_STATUS] = true;
  obj[PARAM_SUCCESS] = true;
  obj[PARAM_MESSAGE] = "Restarting...";
  response->setLength();
  request->send(response);
  _rebootTimer = millis();
  _rebootTask = true;
}

void BaseWebServer::webHandlePing(AsyncWebServerRequest *request) {
  Log.notice(F("WEB : webServer callback for /api/ping." CR));
  AsyncJsonResponse *response = new AsyncJsonResponse(false);
  JsonObject obj = response->getRoot().as<JsonObject>();
  obj[PARAM_STATUS] = true;
  response->setLength();
  request->send(response);
}

void BaseWebServer::setupWebHandlers() {
  if (!_server) return;

  Log.notice(F("WEB : Setting up async web handlers." CR));
  _server->on("/", std::bind(&BaseWebServer::webReturnIndexHtml, this,
                             std::placeholders::_1));
  _server->on("/index.html", std::bind(&BaseWebServer::webReturnIndexHtml, this,
                                       std::placeholders::_1));
  _server->on("/js/app.js", std::bind(&BaseWebServer::webReturnAppJs, this,
                                      std::placeholders::_1));
  _server->on("/css/app.css", std::bind(&BaseWebServer::webReturnAppCss, this,
                                        std::placeholders::_1));
  _server->on("/favicon.ico", std::bind(&BaseWebServer::webReturnFavicon, this,
                                        std::placeholders::_1));

  AsyncCallbackJsonWebHandler *handler;

  _server->on(
      "/api/auth", HTTP_GET,
      std::bind(&BaseWebServer::webHandleAuth, this, std::placeholders::_1));
  _server->on("/api/wifi/status", HTTP_GET,
              std::bind(&BaseWebServer::webHandleWifiScanStatus, this,
                        std::placeholders::_1));
  _server->on("/api/wifi", HTTP_GET,
              std::bind(&BaseWebServer::webHandleWifiScan, this,
                        std::placeholders::_1));
  _server->on(
      "/api/restart", HTTP_GET,
      std::bind(&BaseWebServer::webHandleRestart, this, std::placeholders::_1));
  _server->on(
      "/api/ping", HTTP_GET,
      std::bind(&BaseWebServer::webHandlePing, this, std::placeholders::_1));
  _server->on(
      "/api/filesystem/upload", HTTP_POST,
      std::bind(&BaseWebServer::webReturnOK, this, std::placeholders::_1),
      std::bind(&BaseWebServer::webHandleUploadFile, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4,
                std::placeholders::_5, std::placeholders::_6));
  handler = new AsyncCallbackJsonWebHandler(
      "/api/filesystem",
      std::bind(&BaseWebServer::webHandleFileSystem, this,
                std::placeholders::_1, std::placeholders::_2));
  _server->addHandler(handler);
  _server->on(
      "/api/firmware", HTTP_POST,
      std::bind(&BaseWebServer::webReturnOK, this, std::placeholders::_1),
      std::bind(&BaseWebServer::webHandleUploadFirmware, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4,
                std::placeholders::_5, std::placeholders::_6));
  _server->onNotFound(std::bind(&BaseWebServer::webHandlePageNotFound, this,
                                std::placeholders::_1));
}

bool BaseWebServer::setupWebServer() {
  Log.notice(F("WEB : Configuring web server." CR));

  _server = new AsyncWebServer(80);

  MDNS.begin(_webConfig->getMDNS());
  MDNS.addService("http", "tcp", 80);

#if defined(ESP8266)
  FSInfo fs;
  LittleFS.info(fs);
  Log.notice(F("WEB : File system Total=%d, Used=%d." CR), fs.totalBytes,
             fs.usedBytes);
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    Log.notice(F("WEB : File=%s, %d bytes" CR), dir.fileName().c_str(),
               dir.fileSize());
    if (!dir.fileSize()) {
      Log.notice(F("WEB : Empty file detected, removing file." CR));
      LittleFS.remove(dir.fileName().c_str());
    }
  }
#else
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    Log.notice(F("WEB : File=%s, %d bytes" CR), f.name(), f.size());
    if (!f.size()) {
      Log.notice(F("WEB : Empty file detected, removing file." CR));
      LittleFS.remove(f.name());
    }

    f = root.openNextFile();
  }
  f.close();
  root.close();
#endif

  setupWebHandlers();
  if (_webConfig->isCorsAllowed()) {
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  }

  _server->begin();
  Log.notice(F("WEB : Web server started." CR));
  return true;
}

// EOF
