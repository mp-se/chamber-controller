.. _functionality:

How Chamber Controller works
#############################

Chamber Controller reads temperature sensors, runs the BrewPi temperature
control algorithm, and switches configured heating, cooling, and fan outputs.
The device exposes a web interface over Wi-Fi and can send data to InfluxDB,
BrewGraph, or compatible BLE receivers.

Control modes
=============

The controller offers three modes:

* **Off** — temperature control is disabled.
* **Chamber constant** — regulates the chamber sensor toward the selected
  target. This is useful when the chamber itself should stay at a fixed
  temperature.
* **Beer constant** — regulates the beer temperature toward the selected
  target, using the chamber temperature to manage heating and cooling. This
  mode requires a beer sensor and chamber sensor.

The Home page lets you select an available mode and target temperature. The
firmware stores temperatures internally in Celsius; the selected display unit
only changes how values are shown and entered. Targets are limited to 0–30 °C
(32–86 °F).

Temperature sensors
===================

The chamber sensor is a DS18B20 on the board's OneWire pin. The beer
temperature can come from a second DS18B20 or, on BLE sensor builds, from a
supported BLE temperature device such as GravityMon, Tilt/Tilt Pro, or RAPT
Pill.

A BLE beer sensor needs a chamber sensor as a fallback. When its reading is
older than the configured validity period, the BLE sensor is considered
disconnected and BrewPi PID uses the chamber sensor as the fallback control
input. BLE devices are selected from those discovered by scanning; allow time
for an advertisement before expecting a device to appear.

The selected sensors, offsets, and connection state are visible under
**Device → Controller Hardware** and on the Home page. An unavailable
temperature is shown as disconnected or ``--`` rather than as a valid zero.

Heating and cooling safety
==========================

The controller uses minimum run and rest times, estimated temperature peaks,
and safety cooldown states to avoid rapid or unsafe output changes. A cooling
request may therefore be delayed while a compressor rest timer expires. The
Home page reports states such as **Waiting to cool**, **Waiting to heat**,
**Waiting for peak detect**, and **Cooling safety cooldown**.

These timers are part of the controller algorithm. Do not bypass a compressor's
required rest period by changing wiring or forcing relay outputs. Review
:doc:`configuration` and :doc:`hardware` before connecting mains-powered
equipment.

Web interface
=============

The browser interface is organized into:

* **Home** — current mode, controller state, sensor readings, actuator status,
  Wi-Fi, firmware and integration status.
* **Device** — general settings, sensor and relay configuration, diagnostic
  PID data, and Wi-Fi.
* **Integrations** — Bluetooth temperature broadcasts, InfluxDB v2, and
  BrewGraph reporting/remote control.
* **Other** — firmware update, support information, file and voltage tools,
  serial console, and software details.

The optional TFT displays a compact controller status view. Use the web UI for
configuration and the detailed PID structures.
