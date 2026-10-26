.. _troubleshooting:

Troubleshooting
###############

Web interface cannot find the controller
=========================================

* During first setup, connect to the ``Chamber`` access point and browse to
  ``http://192.168.4.1``.
* After Wi-Fi setup, check that the browser and controller are on the same
  network. Try the configured ``<name>.local`` address or locate the current
  IP in the router's client list.
* If Wi-Fi credentials changed, the IP may have changed. Restart after saving
  the network settings.
* If the interface loads but reports no device response, power-cycle the
  controller and check its serial console.

No sensor readings
==================

* Open **Device → Controller Hardware** and run the sensor scan. Confirm the
  desired OneWire probe appears and is assigned to the correct role.
* Check sensor wiring, OneWire bus power, and the board's OneWire GPIO in
  :doc:`hardware`.
* A BLE sensor must advertise using a supported format and be in range. Enable
  **BLE scan**, select the discovered device, and check its valid-time setting.
* A BLE beer sensor requires a chamber sensor for fallback. The UI blocks
  saving this selection without one.
* Review **Device → PID Controller Info** and the serial log for sensor
  connection information.

Controller mode cannot be selected
==================================

The controller needs at least one configured sensor and at least one enabled
heating or cooling capability. Beer constant mode requires a beer temperature
source; for BLE beer temperature, retain a chamber sensor as fallback. If the
Home page says BrewGraph remote control is active, turn off remote control in
**Integrations → BrewGraph** before using local setpoint controls.

Heating or cooling is delayed
=============================

Check the controller state on Home. Waiting states usually indicate compressor
minimum rest/run timing or peak detection. **Cooling safety cooldown** is a
protective pause after a long cooling run. Check **Device → PID Controller
Info → Min Times** for timing data. Do not bypass these protections by forcing
relay outputs.

Integration is not receiving data
=================================

* **InfluxDB** — check the full URL, network reachability, organization,
  bucket, and write token permissions.
* **BrewGraph** — confirm the complete ingest endpoint and device token. The
  configured interval must respect the five-minute server limit.
* **Bluetooth** — confirm the firmware target supports BLE, then enable
  broadcasting separately from BLE scanning.
* **HTTP client** — inspect **Other → Serial console** for connection and
  response errors.

Firmware update did not complete
================================

Use a firmware file matching the detected platform and hardware. Keep the
controller powered and allow it to restart after upload. If the web interface
cannot recover, follow the board-specific instructions in the web flasher.
Back up configuration before full reflashing when possible.
