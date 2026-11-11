// Host stub, the library only touches the file system when BREWPI_ENABLE_SAVE is defined.
#pragma once
#include <ArduinoJson.h>

class JsonFileSystemHelper {
 public:
  explicit JsonFileSystemHelper(const char*) {}
  bool saveJson(JsonDocument&) { return true; }
  bool loadJson(JsonDocument&) { return false; }
};
