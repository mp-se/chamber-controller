/*
 * Chamber Controller
 * Copyright (c) 2025-2026 Magnus
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
#ifndef SRC_BLE_CHAMBER_HPP_
#define SRC_BLE_CHAMBER_HPP_

#if defined(ENABLE_BLE) && defined(CHAMBER)

#include <NimBLEBeacon.h>
#include <NimBLEDevice.h>

class BleSender {
 private:
  BLEAdvertising* _advertising = nullptr;
  BLEUUID _uuid;
  bool _initFlag = false;
  int _beaconTime = 1000;

  void dumpPayload(const char* payload, int len);

 public:
  BleSender() {}

  void init();

  // Beacons
  void sendCustomBeaconData(float chamberTempC, float beerTempC);
};

#endif  // ENABLE_BLE && CHAMBER

#endif  // SRC_BLE_CHAMBER_HPP_
