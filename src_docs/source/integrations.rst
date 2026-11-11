.. _integrations:

Integrations
############

Chamber Controller can send data to InfluxDB v2, report temperatures to
BrewGraph, broadcast temperatures over BLE, and expose local status through an
HTTP API. Integration availability depends on the selected firmware target.

InfluxDB v2
===========

Open **Integrations → Influxdb v2** and configure:

* **Server** — full InfluxDB URL, including ``http://`` or ``https://`` and
  any required port.
* **Organisation** — organization name or identifier.
* **Bucket** — destination bucket.
* **Authentication token** — a token with write access to that bucket.

Save the settings. While the controller is running, it posts a line-protocol
point every 30 seconds. The point includes the device name and ID, mode, beer
and chamber temperatures and setpoints, cooling and heating actuator states,
and PID state. Confirm that the endpoint is reachable from the controller's
network and that the token can write to the selected organization and bucket.
Protect the token as a secret.

BrewGraph
=========

Open **Integrations → BrewGraph**. This integration reports the chamber and
beer temperatures and receives a control mode and setpoint from the configured
BrewGraph server.

* **Server URL** — full chamber ingest endpoint URL. Leave empty to disable
  reporting.
* **Device token** — token assigned by BrewGraph. If blank, the report contains
  the controller's chip ID instead.
* **Report interval** — select 5, 10, 15, or 20 minutes. The controller makes
  its first poll about 20 seconds after startup, then polls on this interval.
  After a failed poll, it retries at the configured interval. The server
  accepts at most one request per device every five minutes.
* **Allow BrewGraph to control the chamber** — when enabled, the reply can set
  the controller to Beer constant or Chamber constant mode and change its
  target. When disabled, reports still go to BrewGraph, but remote control
  replies are ignored.

When remote control is active, the Home page hides its local mode and setpoint
form and explains that BrewGraph is controlling the device. Turn off the
permission in this integration's settings to return to local control. Remote
setpoints are applied after a successful poll. BrewGraph can return the
controller to local control as well.

Bluetooth
=========

Open **Integrations → Bluetooth** on a BLE-capable build and enable **Enable
temperature sending over bluetooth**. During controller cycles, the device
briefly broadcasts a custom manufacturer-data beacon containing its ID,
chamber temperature, and beer temperature in Celsius. BLE broadcasting is
independent from BLE scanning for a beer sensor; scanning is configured under
**Device → Controller Hardware**. The BLE settings page recommends restarting
after changes.

The controller can also scan for GravityMon, Tilt/Tilt Pro, and RAPT
temperature beacons on supported builds. See :doc:`configuration` for
selecting a BLE beer sensor and handling stale readings.

Home Assistant and other local clients
======================================

The controller exposes a JSON status endpoint at ``/api/status``. A REST
integration can poll this endpoint and select fields such as
``pid_fridge_temp``, ``pid_beer_temp``, ``pid_mode``,
``pid_cooling_actuator_active``, and ``pid_heating_actuator_active``. Sensor
connection fields such as ``pid_fridge_temp_connected`` and
``pid_beer_temp_connected`` should be used to distinguish a missing reading
from a real temperature.
