/*
MIT License

Copyright (c) 2024 Magnus

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
#include <jsonfs.hpp>
#include <log.hpp>
#include <LittleFS.h>

JsonFileSystemHelper::JsonFileSystemHelper(String fileName) {
  _fileName = fileName; 
}

bool JsonFileSystemHelper::saveJson(JsonDocument& doc) {
  File jsonFile = LittleFS.open(_fileName.c_str(), "w");

  if (!jsonFile) {
    Log.error(F("JSFS: Failed to open file %s for save." CR),
              _fileName.c_str());
    return false;
  }

// #if LOG_LEVEL == 6
//   serializeJson(doc, EspSerial);
//   EspSerial.print(CR);
// #endif
  serializeJson(doc, jsonFile);
  jsonFile.flush();
  jsonFile.close();
  Log.notice(F("CFG : Data saved to file." CR));
  return true;
}

bool JsonFileSystemHelper::loadJson(JsonDocument& doc) {
  Log.verbose(F("CFG : Loading data from file %s." CR), _fileName.c_str());

  if (!LittleFS.exists(_fileName.c_str())) {
    Log.error(F("CFG : Configuration file %s does not exist." CR),
              _fileName.c_str());
    return false;
  }

  File jsonFile = LittleFS.open(_fileName.c_str(), "r");

  if (!jsonFile) {
    Log.error(F("CFG : Failed to open %s." CR), _fileName.c_str());
    return false;
  }

  DeserializationError err = deserializeJson(doc, jsonFile);
// #if LOG_LEVEL == 6
//   serializeJson(doc, EspSerial);
//   EspSerial.print(CR);
// #endif
  jsonFile.close();

  if (err) {
    Log.error(F("CFG : Failed to parse file, Err: %s." CR), err.c_str());
    return false;
  }

  Log.notice(F("CFG : Json file %s loaded." CR), _fileName.c_str());
  return true;
}

// EOF
