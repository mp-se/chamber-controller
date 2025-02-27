/*
MIT License

Copyright (c) 2025 Magnus

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
#if defined(ENABLE_BLE) && defined(CHAMBER)

#include <ble_chamber.hpp>
#include <log.hpp>
#include <string>

// Tilt UUID variants and data format, based on tilt-sim
//
// https://github.com/spouliot/tilt-sim
//
// Tilt data format is described here. Only SG and Temp is transmitted over BLE.
// https://kvurd.com/blog/tilt-hydrometer-ibeacon-data-format/

void BleSender::init() {
  if (_initFlag) return;

  BLEDevice::init("chamber");
  _advertising = BLEDevice::getAdvertising();

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

  _initFlag = true;
}

void BleSender::sendCustomBeaconData(float chamberTempC, float beerTempC) {
  Log.info(F("Starting custom beacon data transmission" CR));

  _advertising->stop();

  uint16_t c = chamberTempC * 1000;
  uint16_t b = beerTempC * 1000;
  uint32_t chipId = 0;

  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  std::string mf = "";

  mf += static_cast<char>(0x4C);  // Manuf ID (Apple)
  mf += static_cast<char>(0x00);
  mf += static_cast<char>(0x03);  // SubType (standards is 0x02)
  mf += static_cast<char>(0x15);  // SubType Length
  mf += "CHAMBER.";
  mf += static_cast<char>(((chipId & 0xFF000000) >> 24));  // Chipid
  mf += static_cast<char>(((chipId & 0xFF0000) >> 16));
  mf += static_cast<char>(((chipId & 0xFF00) >> 8));
  mf += static_cast<char>((chipId & 0xFF));
  mf += static_cast<char>((c >> 8));  // Chamber Temp
  mf += static_cast<char>((c & 0xFF));
  mf += static_cast<char>((b >> 8));  // Beer Temp
  mf += static_cast<char>((b & 0xFF));  
  mf += static_cast<char>(0x00);
  mf += static_cast<char>(0x00);
  mf += static_cast<char>(0x00);
  mf += static_cast<char>(0x00);
  mf += static_cast<char>(0x00);  // Signal

#if LOG_LEVEL == 6
  dumpPayload(mf.c_str(), mf.length());
#endif

  BLEAdvertisementData advData = BLEAdvertisementData();
  advData.setFlags(0x04);
  advData.setManufacturerData(mf);
  _advertising->setAdvertisementData(advData);

  _advertising->setConnectableMode(BLE_GAP_CONN_MODE_NON);
  _advertising->start();
  delay(_beaconTime);
  _advertising->stop();
}

void BleSender::dumpPayload(const char* p, int len) {
  for (int i = 0; i < len; i++) {
    EspSerial.printf("%X%X ", (*(p + i) & 0xf0) >> 4, (*(p + i) & 0x0f));
  }
  EspSerial.println();
}

#endif  // ENABLE_BLE && CHAMBER
