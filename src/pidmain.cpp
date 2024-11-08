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
#include <espframework.hpp>
#include <log.hpp>
#include <looptimer.hpp>
#include <pidconfig.hpp>
#include <pidmain.hpp>
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

void setup() {
  delay(2000);

  Log.notice(F("Main: Started setup." CR));

  myConfig.checkFileSystem();
  myConfig.loadFile();

  myWifi.init();
  if (!myWifi.hasConfig() || myWifi.isDoubleResetDetected()) {
    Log.notice(
        F("Main: Missing wifi config or double reset detected, entering wifi "
          "setup." CR));
    myWebServer.setWifiSetup(true);
    myWifi.startAP();
  } else {
    myConfig.setWifiScanAP(true);
    myWifi.connect(WIFI_AP_STA);
    myWifi.timeSync();
  }

  myWebServer.setupWebServer();
  mySerialWebSocket.begin(myWebServer.getWebServer(), &EspSerial);
  mySerial.begin(&mySerialWebSocket);

  oneWire.begin(Config::Pins::oneWirePin);

  configureTempControl();

  Log.notice(F("Main: Setup is completed." CR));
}

LoopTimer intLoop(1000);
LoopTimer pushLoop(5000);

void loop() {
  if(!myWifi.isConnected()) {
    Log.notice(F("Main: No wifi connection, attempting to reconnect." CR));
    myWifi.connect(WIFI_AP_STA);
    myWifi.timeSync();
  }

  if (intLoop.hasExipred()) {
    intLoop.reset();
    tempControl.loop();
  }

  if (pushLoop.hasExipred()) {
    pushLoop.reset();

    if(myConfig.hasTargetInfluxDb2()) {
      char buf[250];

      snprintf(buf, sizeof(buf),
               "chamber,host=%s,device=%s,mode=%c "
               "beer-temp=%f,"
               "chamber-temp=%f,"
               "beer-target-temp=%f,"
               "chamber-target-temp=%f,"
               "actuator-cool=%d,"
               "actuator-heat=%d",
               myConfig.getMDNS(), myConfig.getID(), tempControl.getMode(), tempControl.getBeerTemperature(), tempControl.getFridgeTemperature(),
               tempControl.getBeerTemperatureSetting(), tempControl.getFridgeTemperatureSetting(), tempControl.getCoolingActuator()->isActive() ? 1 : 0, 
               tempControl.getHeatingActuator()->isActive() ? 1 : 0);

      Log.notice(F("Main: Pushing data to influxdb2." CR));
      // EspSerial.println(buf);

      String s = buf;
      myPush.sendInfluxDb2(s);
    }
  }

  myUptime.calculate();
  myWifi.loop();
  myWebServer.loop();
  mySerialWebSocket.loop();
}

void configureTempControl() {
  // Set controller to default; mode, actuators and sensors
  Log.notice(F("Main: Initializing temp control." CR));
  tempControl.init(MIN_TIMES_DEFAULT);

  // Create the temperature sensors
  if (myConfig.isFridgeSensorEnabled()) {
    Log.notice(F("Main: Configuring fridge sensor %s." CR),
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
    Log.notice(F("Main: Configuring beer sensor %s." CR),
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
    actuatorCooling = new DigitalPinActuator(Config::Pins::coolingPin, false);
  }

  if (!actuatorHeating) {
    actuatorHeating = new DigitalPinActuator(Config::Pins::heatingPin, false);
  }

  if (myConfig.isCoolingEnabled()) {
    Log.notice(F("Main: Configuring cooling actuator." CR));
    tempControl.setCoolingActuator(actuatorCooling);
  }

  if (myConfig.isHeatingEnabled()) {
    Log.notice(F("Main: Configuring cooling actuator." CR));
    tempControl.setHeatingActuator(actuatorHeating);
  }

  if (myConfig.isControllerFridgeConstant()) {
    Log.notice(F("Main: Setting fridge target temperature %F." CR),
               myConfig.getTargetTemperature());
    tempControl.setFridgeTargetTemperature(myConfig.getTargetTemperature());
  } else if (myConfig.isControllerBeerConstant()) {
    Log.notice(F("Main: Setting beer target temperature %F." CR),
               myConfig.getTargetTemperature());
    tempControl.setBeerTargetTemperature(myConfig.getTargetTemperature());
  }

  Log.notice(F("Main: Setting mode %c." CR), myConfig.getControllerMode());
  tempControl.setMode(myConfig.getControllerMode());
}

// EOF