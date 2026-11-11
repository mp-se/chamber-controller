.. _hardware:

Hardware
########

Supported targets
-----------------

The repository includes these firmware environments:

* **Lolin ESP32 D32 Pro** with a 2.4-inch TFT. BLE temperature scanning and
  BLE broadcast are enabled.
* **Lolin ESP32-S2 Mini** without a TFT or BLE support in the current build.
* **Lolin ESP32-S3 Mini** without a TFT. BLE temperature scanning and
  broadcasting are enabled.

Use a firmware image built for the matching chip and board. Hardware features
shown in the web interface depend on the build; for example, BLE settings are
not available when BLE support is absent.

Sensor and actuator pins
------------------------

.. list-table:: Sensor and actuator pins
   :header-rows: 1

   * - Board
     - Heating
     - Cooling
     - Fan
     - OneWire
   * - ESP32 PRO
     - GPIO 25
     - GPIO 26
     - GPIO 15
     - GPIO 13
   * - ESP32-S2 Mini
     - GPIO 5
     - GPIO 7
     - GPIO 3
     - GPIO 9
   * - ESP32-S3 Mini
     - GPIO 4
     - GPIO 12
     - GPIO 2
     - GPIO 13

The OneWire bus supports DS18B20 probes. The normal configuration selects
probes by their discovered sensor address, so label physical probes during
installation and confirm which reading belongs to each position. Connect
OneWire sensors with the required pull-up and power arrangement for the probe
and board.

The heating, cooling, and fan GPIOs are control signals for suitable relay or
driver circuitry. They are not mains outputs. Use an electrically appropriate
isolated driver, protect the load, and follow the relay and appliance ratings.
The **Invert pins** option changes the active logic level, not the electrical
isolation or load rating. Confirm the outputs with the equipment disconnected
before connecting a heater or compressor.

Display
-------

Only the ESP32 PRO target in the current PlatformIO configuration enables the
2.4-inch ILI9341 TFT and LVGL interface. The display shows controller state,
chamber and beer temperatures, target temperature, and Wi-Fi status. The
browser remains the main interface for configuration, integrations, and PID
diagnostics. The S2 and S3 Mini targets use the browser interface without an
integrated display.

Power and placement
-------------------

Use a stable supply appropriate for the selected development board, TFT, and
attached sensors. Keep the controller and temperature probes away from
unintended heat sources. Place the chamber probe where it measures chamber
air representative of the controlled space and secure the beer probe to the
fermenter as appropriate for the installation.

Board definitions, PlatformIO build flags, and pin assignments in
``platformio.ini`` and ``lib/brewpi-pid/Config.hpp`` are the source of truth
when building for a custom board. Confirm that the actual board revision
matches those definitions before wiring outputs.
