/*
 * Chamber Controller
 * Copyright (c) 2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef SRC_INTEGRATIONSTATUS_HPP_
#define SRC_INTEGRATIONSTATUS_HPP_

#include <Arduino.h>

#include <basepush.hpp>

// Shared last-request state for outbound integrations. The API representation
// deliberately matches Kegmon: push_used, push_age, push_status, push_code,
// and push_response.
class IntegrationStatus {
 private:
  bool _hasRun = false;
  uint32_t _lastTimestamp = 0;
  bool _lastStatus = false;
  int _lastError = 0;
  String _lastResponse;

 public:
  void update(const BasePush& push, const String& response) {
    _lastTimestamp = millis();
    _lastStatus = push.wasLastSuccessful();
    _lastError = push.getLastResponseCode();
    _lastResponse = response.substring(0, 160);
    _hasRun = true;
  }

  bool hasRun() const { return _hasRun; }
  uint32_t getLastTimestamp() const { return _lastTimestamp; }
  bool getLastStatus() const { return _lastStatus; }
  int getLastError() const { return _lastError; }
  const String& getLastResponse() const { return _lastResponse; }
};

#endif  // SRC_INTEGRATIONSTATUS_HPP_

// EOF
