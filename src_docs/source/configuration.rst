.. _configuration:

Configuration
#############

Open the controller's mDNS address (``<name>.local``) or IP address in a
browser. The navigation groups settings under **Device**, **Integrations**,
and **Other**. Save a page after changing values; settings with network,
Bluetooth, or hardware effects may need a restart.

Home page
=========

The Home page refreshes status every few seconds. It shows the selected
controller mode and target, state and wait timers, chamber and beer
temperatures, selected sensors, heating/cooling/fan activity, BLE devices (on
supported builds), network signal and address, memory, firmware, and
integration status.

The controller card can set **Off**, **Chamber constant**, or **Beer constant**
and a target temperature. The interface accepts 0–30 °C; Fahrenheit is
converted for display and entry. The firmware rejects a constant mode if its
required sensor or both heating and cooling outputs are unavailable. Beer
constant mode also requires a chamber sensor when the beer sensor is BLE. If
BrewGraph remote control is active, the local mode and target controls are
replaced by a notice; disable remote control on the BrewGraph page to return
to local control.

Device settings
===============

Open **Device → Settings**.

* **mDNS** — network name used for ``<name>.local``. Enter a unique name; the
  ``.local`` suffix is added automatically.
* **Temperature Format** — choose Celsius or Fahrenheit for the interface. The
  control algorithm continues to use Celsius internally.
* **User Interface** — choose day or dark appearance.
* **Restart interval** — optional automatic restart interval: disabled, 30
  minutes, 1, 2, 4, 6, 12, or 24 hours. This is available for stability
  management; leave disabled unless a periodic restart is useful for the
  installation.
* **Save**, **Restart device**, and **Restore factory defaults** — save
  settings, restart, or erase configuration and restore defaults. Export or
  record settings before a factory reset.

Controller hardware and sensors
===============================

Open **Device → Controller Hardware**. The page scans the OneWire bus and lists
detected sensors. A previously selected sensor that is not currently detected
is retained in the list and marked as not detected.

* **Chamber Sensor** — select the DS18B20 measuring the chamber or fridge.
* **Beer Sensor** — select a second OneWire DS18B20 for beer temperature, or
  leave it unset when using a BLE beer sensor.
* **Fridge Sensor Offset / Beer Sensor Offset** — temperature correction in
  degrees, from −5 to +5 in 0.01 degree increments. Use a measured reference
  temperature to determine a correction.
* **Enable Cooling / Heating / Fan** — enable the corresponding output. The
  required sensor and at least one heating or cooling capability must be
  configured before selecting a control mode.
  When enabled, the fan output runs while either heating or cooling is active.
* **Invert pins** — reverses the active electrical level of the outputs. Set
  this to match the relay/driver board; verify output behavior without a live
  mains load.
* **BLE scan** — enables scanning for BLE temperature sensors, when available
  in the firmware.
* **BLE valid time** — age in minutes after which a BLE reading is considered
  disconnected (5–30 minutes). The controller then falls back to the chamber
  sensor for the beer-control input.
* **BLE Beer Sensor** — select a discovered BLE temperature sensor as the beer
  input. A chamber sensor is still required for fallback while the BLE sensor
  is unavailable.

After saving sensor or output changes, confirm readings and state on Home. A
BLE beer sensor can take a few advertising cycles to appear in the selection
list. This firmware recognizes GravityMon, Tilt/Tilt Pro, and RAPT temperature
beacons. BLE scanning and sensor selection require a build with both BLE and
BLE sensor support.

PID controller data
===================

Open **Device → PID Controller Info** to inspect four internal structures:

* **Control Constants** — configured algorithm constants.
* **Control Settings** — learned and stored controller settings.
* **Control Variables** — current runtime variables.
* **Min Times** — minimum run/rest and safety timing parameters.

Use **Reload** to fetch the selected structure again. These data are primarily
for troubleshooting and controller tuning; change them only when you
understand the BrewPi PID algorithm and have recorded the original values.

Wi-Fi
=====

Open **Device → Wifi** to select one or two networks. The page scans nearby
access points and indicates signal strength and whether a network is secured.
At least one network must be selected. If an SSID is hidden, follow the manual
entry link to **Device → Wifi** manual entry.

* **SSID #1 / Password #1** — primary network.
* **SSID #2 / Password #2** — optional alternate network.
* **Portal timeout** — setup portal idle timeout, 10–240 seconds.
* **Connection timeout** — wait for a network connection, 1–60 seconds.
* **Scan for strongest AP**, when shown — scan and select the strongest access
  point for the configured network; this can lengthen connection time.

Save changed credentials and restart. Reconnect to the controller at its new
IP address or mDNS name if it changes.

Integrations
============

Integration settings are under **Integrations**. See :doc:`integrations` for
the field-by-field details and setup notes for Bluetooth, InfluxDB, and
BrewGraph.

Maintenance
===========

Under **Other**, use **Firmware update** for an in-place firmware upload,
**Serial console** to inspect live logs, **Support** for project links and
device logs, and **Tools** for filesystem inspection and optional advanced
operations. The PID details page and serial log are useful when sensor or
controller states do not match expectations.
