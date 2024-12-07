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
#include <OneWire.h>

#include <ActuatorDigitalPin.hpp>
#include <Config.hpp>
#include <NumberFormats.hpp>
#include <TempControl.hpp>
#include <TempSensorOneWire.hpp>
#include <display.hpp>
#include <espframework.hpp>
#include <log.hpp>
#include <looptimer.hpp>
#include <main.hpp>
#include <pidconfig.hpp>
#include <pidpush.hpp>
#include <pidwebserver.hpp>
#include <serialws.hpp>
#include <uptime.hpp>
#include <wificonnection.hpp>

SerialDebug mySerial(115200L);
PidConfig myConfig("chamber", "/chamber.cfg");
WifiConnection myWifi(&myConfig, "chamber", "password", "Chamber Controller",
                      "", "");
PidPush myPush(&myConfig);

PidWebServer myWebServer(&myConfig, &myPush);
SerialWebSocket mySerialWebSocket;

OneWire oneWire;
DigitalPinActuator *actuatorCooling = NULL;
DigitalPinActuator *actuatorHeating = NULL;
OneWireTempSensor *oneWireFridge = NULL;
OneWireTempSensor *oneWireBeer = NULL;
TempSensor *fridgeSensor = NULL;
TempSensor *beerSensor = NULL;
Display myDisplay;

RunMode runMode = RunMode::pidMode;

void setup() {
  delay(2000);

  Log.notice(F("Main: Started setup." CR));

  myConfig.checkFileSystem();
  myConfig.loadFile();

  // PINS: MISO=-1, MOSI=23, SCLK=18, CS=14, DC=27, RST=33, TOUCH_DS=12

  Log.notice(F("Main: Initialize display." CR));
  myDisplay.setup();
  myDisplay.setFont(FontSize::FONT_12);
  myDisplay.printLineCentered(1, "Chamber Controller");
  myDisplay.printLineCentered(2, "Starting");

#if !defined(WOKWI)
  myDisplay.calibrateTouch();
  myDisplay.printLineCentered(2, "Connecting to WIFI");

  myWifi.init();
  if (!myWifi.hasConfig() || myWifi.isDoubleResetDetected()) {
    runMode = RunMode::wifiSetupMode;
    Log.notice(
        F("Main: Missing wifi config or double reset detected, entering wifi "
          "setup." CR));
    myDisplay.printLineCentered(2, "Entering WIFI setup");
    myWebServer.setWifiSetup(true);
    myWifi.startAP();
  } else {
    myConfig.setWifiScanAP(true);
    myWifi.connect();
    myWifi.timeSync();
  }
#else
  myConfig.setWifiSSID("Wokwi-GUEST", 0);
  myConfig.setWifiPass("", 0);
  myConfig.setFridgeSensorId("123456");
  myConfig.setCoolingEnabled(true);

  myWifi.connect(WIFI_AP_STA);
  myWifi.timeSync();
#endif

  myWebServer.setupWebServer();
  mySerialWebSocket.begin(myWebServer.getWebServer(), &EspSerial);
  mySerial.begin(&mySerialWebSocket);
  oneWire.begin(Config::Pins::oneWirePin);

  switch (runMode) {
    case RunMode::pidMode:
      myDisplay.printLineCentered(2, "Configuring Temp Control");

      // Configure temp controller
      configureTempControl();

      // Create graphical UI
      myDisplay.createUI();
      myDisplay.setTargetTemperature(myConfig.getTargetTemperature());
      myDisplay.setMode(myConfig.getControllerMode());
      break;

    case RunMode::wifiSetupMode:
      break;
  }

  Log.notice(F("Main: Setup is completed." CR));
}

LoopTimer tempControlLoop(1000);
LoopTimer validateLoop(10000);
LoopTimer pushLoop(30000);

void runLoop() {
  if (!myWifi.isConnected()) {
    Log.notice(F("Loop: No wifi connection, attempting to reconnect." CR));
    myWifi.connect(WIFI_AP_STA);
    myWifi.timeSync();
  }

  if (tempControlLoop.hasExipred()) {
    tempControlLoop.reset();

    Log.verbose(F("Loop: Running temp control." CR));

    myDisplay.updateButtons(strlen(myConfig.getBeerSensorId()),
                            strlen(myConfig.getFridgeSensorId()));

    float beer = NAN, fridge = NAN;

    if (!tempControl.isDefaultBeerSensor() &&
        tempControl.getBeerSensor()->isConnected()) {
      beer = tempControl.getBeerTemperature();
    }

    if (!tempControl.isDefaultFridgeSensor() &&
        tempControl.getFridgeSensor()->isConnected()) {
      fridge = tempControl.getFridgeTemperature();
    }

    char state[40] = "", mode[40] = "Off", statusBar[80] = "";

    switch (tempControl.getMode()) {
      case ControllerMode::beerConstant:
        snprintf(mode, sizeof(mode), "Beer -> %.1F°%c",
                 tempControl.getBeerTemperatureSetting(),
                 myConfig.getTempFormat());
        break;
      case ControllerMode::fridgeConstant:
        snprintf(mode, sizeof(mode), "Chamber -> %.1F°%c",
                 tempControl.getFridgeTemperatureSetting(),
                 myConfig.getTempFormat());
        break;
    }

    switch (tempControl.getState()) {
      case ControllerState::IDLE: {
        uint16_t t =
            tempControl.timeSinceCooling() < tempControl.timeSinceHeating()
                ? tempControl.timeSinceCooling()
                : tempControl.timeSinceHeating();

        if(t>3600)
          snprintf(state, sizeof(state), "Idle %02dh %02dm", t / 3600, (t % 3600) / 60);
        else
          snprintf(state, sizeof(state), "Idle %02dm %02ds", t / 60, t % 60);
      } break;
      case ControllerState::STATE_OFF:
        break;
      case ControllerState::HEATING: {
        if(tempControl.timeSinceIdle()>3600)
          snprintf(state, sizeof(state), "Heating %02dh %02dm",
                  tempControl.timeSinceIdle() / 3600,
                  (tempControl.timeSinceIdle() % 3600) / 60);
        else
          snprintf(state, sizeof(state), "Heating %02dm %02ds",
                  tempControl.timeSinceIdle() / 60,
                  tempControl.timeSinceIdle() % 60);
      } break;
      case ControllerState::COOLING: {
        if(tempControl.timeSinceIdle()>3600)
          snprintf(state, sizeof(state), "Cooling %02dh %02dm",
                  tempControl.timeSinceIdle() / 3600,
                  (tempControl.timeSinceIdle() % 3600) / 60);
        else
          snprintf(state, sizeof(state), "Cooling %02dm %02ds",
                  tempControl.timeSinceIdle() / 60,
                  tempControl.timeSinceIdle() % 60);
      } break;
      case ControllerState::COOLING_MIN_TIME:
      case ControllerState::HEATING_MIN_TIME: {
        if(tempControl.timeSinceIdle()>3600)
          snprintf(state, sizeof(state), "Waiting %02dh %02dm",
                  tempControl.timeSinceIdle() / 3600,
                  (tempControl.timeSinceIdle() % 3600) / 60);
        else
          snprintf(state, sizeof(state), "Waiting %02dm %02ds",
                  tempControl.timeSinceIdle() / 60,
                  tempControl.timeSinceIdle() % 60);
      } break;

      case ControllerState::WAITING_TO_HEAT:
      case ControllerState::WAITING_TO_COOL:
      case ControllerState::WAITING_FOR_PEAK_DETECT: {
        if(tempControl.timeSinceIdle()>3600)
          snprintf(state, sizeof(state), "Waiting %02dh %02dm",
                  tempControl.getWaitTime() / 3600,
                  (tempControl.getWaitTime() % 3600) / 60);
        else
          snprintf(state, sizeof(state), "Waiting %02dm %02ds",
                  tempControl.getWaitTime() / 60,
                  tempControl.getWaitTime() % 60);
      } break;
    }

    if (!myWifi.isConnected()) {
      snprintf(statusBar, sizeof(statusBar), "Not connected");
    } else {
      snprintf(statusBar, sizeof(statusBar), "ssid: %s ip: %s, rssi: %d",
               WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
               WiFi.RSSI());
    }

    myDisplay.updateTemperatures(mode, state, statusBar, beer, fridge,
                                 myConfig.getTempFormat());
    tempControl.loop();
  }

  // if (validateLoop.hasExipred()) {
  //   validateLoop.reset();
  //   validateTempControl();
  // }

  if (pushLoop.hasExipred()) {
    pushLoop.reset();

    if (myConfig.hasTargetInfluxDb2()) {
      char buf[250];

      snprintf(buf, sizeof(buf),
               "chamber,host=%s,device=%s,mode=%c "
               "beer-temp=%f,"
               "chamber-temp=%f,"
               "beer-target-temp=%f,"
               "chamber-target-temp=%f,"
               "actuator-cool=%d,"
               "actuator-heat=%d,"
               "state=%d",
               myConfig.getMDNS(), myConfig.getID(), tempControl.getMode(),
               tempControl.getBeerTemperature(),
               tempControl.getFridgeTemperature(),
               tempControl.getBeerTemperatureSetting(),
               tempControl.getFridgeTemperatureSetting(),
               tempControl.getCoolingActuator()->isActive() ? 1 : 0,
               tempControl.getHeatingActuator()->isActive() ? 1 : 0,
               tempControl.getState());

      Log.info(F("Main: Pushing data to influxdb2." CR));
      // EspSerial.println(buf);

      String s = buf;
      myPush.sendInfluxDb2(s);
    }
  }
}

void setNewControllerMode(char mode, float temp) {
  Log.info(F("Main: New mode/temperature set %c %F." CR), mode, temp);

  myConfig.setControllerMode(mode);
  myConfig.setTargetTemperature(temp);
  myConfig.saveFile();

  tempControl.setMode(mode);
  if (mode == ControllerMode::beerConstant)
    tempControl.setBeerTargetTemperature(temp);
  else if (mode == ControllerMode::fridgeConstant)
    tempControl.setFridgeTargetTemperature(temp);
}

void loop() {
  switch (runMode) {
    case RunMode::pidMode:
      runLoop();
      break;

    case RunMode::wifiSetupMode:
      break;
  }

  myUptime.calculate();
  myWifi.loop();
  myWebServer.loop();
  mySerialWebSocket.loop();
}

void validateTempControl() {
  /*
  // Check that sensors and control mode is correct
  Log.info(F("Main: Validate temp control settings." CR));

  bool needInitialize = false;

  // Check cooling actuator if settings are correct
  if (myConfig.isCoolingEnabled() && tempControl.isDefaultCoolingActator()) {
    Log.warning(F("Main: Cooling actutor is enabled but not configured!" CR));
    needInitialize = true;
  }

  // Check heating actuator if settings are correct
  if (myConfig.isHeatingEnabled() && tempControl.isDefaultHeatingActator()) {
    Log.warning(F("Main: Heating actutor is enabled but not configured!" CR));
    needInitialize = true;
  }

  bool fridge = tempControl.getFridgeSensor()->isConnected();
  bool beer = tempControl.getBeerSensor()->isConnected();

  // Check fridge sensor if settings are correct
  if (myConfig.isFridgeSensorEnabled() && fridge) {
    Log.warning(
        F("Main: Fridge sensor is enabled but sensor is not connected!" CR));
    needInitialize = true;
  }

  // Check beer sensor if settings are correct
  if (myConfig.isBeerSensorEnabled() && beer) {
    Log.warning(
        F("Main: Beer sensor is enabled but sensor is not connected!" CR));
    needInitialize = true;
  }

  switch (myConfig.getControllerMode()) {
    case ControllerMode::off:
      break;
    case ControllerMode::beerConstant:
      // TODO: Add validation options
      break;
    case ControllerMode::fridgeConstant:
      // TODO: Add validation options
      break;
  }

  if (needInitialize) {
    configureTempControl();
  }*/
}

void configureTempControl() {
  // Set controller to default; mode, actuators and sensors
  Log.info(F("Main: Initializing temp control." CR));
  tempControl.init(MIN_TIMES_DEFAULT);

  if(!myConfig.isFridgeSensorEnabled() && !myConfig.isBeerSensorEnabled()) {
    Log.error(F("Main: No sensor configured, unable to configure Temp Controller." CR));
    return;
  }

  if(!myConfig.isCoolingEnabled() && !myConfig.isHeatingEnabled()) {
    Log.error(F("Main: Heating or Cooling is not configured, unable to configure Temp Controller." CR));
    return;
  }

  // Create the temperature sensors
  if (myConfig.isFridgeSensorEnabled()) {
    Log.info(F("Main: Configuring fridge sensor %s." CR),
             myConfig.getFridgeSensorId());

    DeviceAddress daFridge;
    parseBytes(daFridge, myConfig.getFridgeSensorId(), sizeof(daFridge));

    if (oneWireFridge) delete oneWireFridge;
    if (fridgeSensor) delete fridgeSensor;

    oneWireFridge =
        new OneWireTempSensor(&oneWire, daFridge, 0);  // TODO: Configure offset
    fridgeSensor = new TempSensor(TEMP_SENSOR_TYPE_FRIDGE, oneWireFridge);
    fridgeSensor->init();
    tempControl.setFridgeSensor(fridgeSensor);
  }

  if (myConfig.isBeerSensorEnabled()) {
    Log.info(F("Main: Configuring beer sensor %s." CR),
             myConfig.getBeerSensorId());

    DeviceAddress daBeer;
    parseBytes(daBeer, myConfig.getBeerSensorId(), sizeof(daBeer));

    if (oneWireBeer) delete oneWireBeer;
    if (beerSensor) delete beerSensor;

    oneWireBeer =
        new OneWireTempSensor(&oneWire, daBeer, 0);  // TODO: Configure offset
    beerSensor = new TempSensor(TEMP_SENSOR_TYPE_BEER, oneWireBeer);
    beerSensor->init();
    tempControl.setBeerSensor(beerSensor);
  }

  // Create the actuators
  if (!actuatorCooling) {
    actuatorCooling = new DigitalPinActuator(Config::Pins::coolingPin,
                                             myConfig.isPinsInverted());
  }

  if (!actuatorHeating) {
    actuatorHeating = new DigitalPinActuator(Config::Pins::heatingPin,
                                             myConfig.isPinsInverted());
  }

  if (myConfig.isCoolingEnabled()) {
    Log.info(F("Main: Configuring cooling actuator." CR));
    tempControl.setCoolingActuator(actuatorCooling);
  }

  if (myConfig.isHeatingEnabled()) {
    Log.info(F("Main: Configuring cooling actuator." CR));
    tempControl.setHeatingActuator(actuatorHeating);
  }

  if (myConfig.isControllerFridgeConstant()) {
    Log.info(F("Main: Setting fridge target temperature %F." CR),
             myConfig.getTargetTemperature());
    tempControl.setFridgeTargetTemperature(myConfig.getTargetTemperature());
  } else if (myConfig.isControllerBeerConstant()) {
    Log.info(F("Main: Setting beer target temperature %F." CR),
             myConfig.getTargetTemperature());
    tempControl.setBeerTargetTemperature(myConfig.getTargetTemperature());
  }

#if defined(BREWPI_ENABLE_SAVE)
  tempControl.loadConstants();
  tempControl.loadSettings();
#endif

  if(myConfig.getControllerMode() != tempControl.getMode()) { // If we have a missmatch after loading settings, set the controller mode 
#if defined(BREWPI_ENABLE_SAVE)
    Log.warning(F("Main: Stored controller mode does not match configured, updating." CR));
#endif
    Log.info(F("Main: Setting mode %c." CR), myConfig.getControllerMode());
    tempControl.setMode(myConfig.getControllerMode());
  }
}

// EOF
