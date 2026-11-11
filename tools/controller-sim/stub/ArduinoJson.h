// Host stub of ArduinoJson that only RECORDS numeric values written with obj["key"] = number,
// so a test can read the controller's internal variables through toJsonReadable().
#pragma once
#include <Arduino.h>

#include <map>
#include <string>
#include <type_traits>

using JsonRecord = std::map<std::string, double>;

struct JsonVariant {
  JsonRecord* rec = nullptr;
  std::string key;

  template <class T, typename std::enable_if<std::is_arithmetic<T>::value || std::is_enum<T>::value, int>::type = 0>
  JsonVariant& operator=(T v) {
    if (rec) (*rec)[key] = static_cast<double>(v);
    return *this;
  }
  JsonVariant& operator=(const String&) { return *this; }
  JsonVariant& operator=(const char*) { return *this; }

  template <class T> bool is() const { return false; }
  template <class T> T as() const { return T(); }
  bool isNull() const { return true; }
  template <class T, typename std::enable_if<std::is_arithmetic<T>::value || std::is_enum<T>::value, int>::type = 0>
  operator T() const {
    return T();
  }
};

struct JsonObject {
  JsonRecord* rec = nullptr;
  JsonVariant operator[](const char* key) const {
    JsonVariant v;
    v.rec = rec;
    v.key = key;
    return v;
  }
};

struct JsonDocument {
  JsonRecord record;
  JsonVariant operator[](const char* key) {
    JsonVariant v;
    v.rec = &record;
    v.key = key;
    return v;
  }
  template <class T> T as() {
    T t;
    t.rec = &record;
    return t;
  }
  template <class T> T to() {
    T t;
    t.rec = &record;
    return t;
  }
};

template <class T> inline T serialized(T v) { return v; }
