.. _installation:

Installation and first setup
############################

Choose firmware for the exact board. The project provides ESP32 PRO, ESP32-S2
Mini, and ESP32-S3 Mini builds; see :doc:`hardware` for display, BLE, and pin
differences. For routine updates, use the device's **Other → Firmware update**
page. For initial installation or recovery, use the `Gravitymon Web flasher <https://gravitymon.com/flasher/index.html>`_ and follow its board
and flash instructions.

First Wi-Fi setup
=================

When Wi-Fi credentials have not been configured, the controller starts a
temporary access point named ``Chamber`` with password ``password``. Connect a
phone or computer to that network and open ``http://192.168.4.1``. In
**Device → Wifi**, choose a network and enter its password, then save and
restart. The same setup access point can be entered by triggering the board's
double-reset detection. If the network is hidden, use the manual entry link on
the Wi-Fi page.

After connecting to the home network, find the address on the TFT (if fitted),
in the router's client list, or by using the configured mDNS name
``<name>.local``. Open that address in a browser. Network changes can change
the controller's IP address.

Initial configuration
======================

1. Open **Device → Settings** and set a unique mDNS name and temperature unit.
2. Open **Device → Controller Hardware**, scan for OneWire sensors, and select
   the chamber and beer sensors.
3. Select heating, cooling, and fan outputs that match the connected hardware.
4. Save the hardware configuration and verify that the Home page reports the
   expected sensor readings and actuator states.
5. On the Home page, select a control mode and target temperature.
6. Configure optional integrations under **Integrations**.

Test sensor readings and output wiring before connecting a heater, compressor,
or other mains-powered load. See :doc:`hardware` for board pins and safety
notes.

Firmware updates
================

For a web update, open **Other → Firmware update**, confirm the detected
platform and hardware, select the matching firmware ``.bin`` file, and start
the upload. Keep the controller powered while the upload and restart complete.
For board-specific full flash images or flash layout changes, use the web
flasher and its matching board package. Back up the configuration before a
reset or full reinstall where possible.
