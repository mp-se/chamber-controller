/*
GNU GENERAL PUBLIC LICENSE
Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.

Based on code created by John Beeler on (BREWPI-ESP8266 project).
 */
#ifndef SRC_DALLASTEMPNG_HPP_
#define SRC_DALLASTEMPNG_HPP_

#include <DallasTemperature.h>
#include <OneWire.h>

bool initConnection(DallasTemperature& sensor, const uint8_t* deviceAddress);
int16_t getTempRaw(DallasTemperature& sensor, const uint8_t* deviceAddress);

#endif  // SRC_DALLASTEMPNG_HPP_

// EOF
