.. _releases:

Release notes
#############

0.9.0
=====

* Added BrewGraph integration. The controller reports beer and chamber temperature, mode and target to BrewGraph at a configurable interval (5, 10, 15 or 20 minutes) and can optionally let BrewGraph set the mode and target temperature (remote control).
* Added integration status for BrewGraph on the home page, showing the result of the last request.
* Remote control is now clearly indicated. The web interface shows a Remote badge and an explanation in the controller card, and the TFT status bar starts with REMOTE while BrewGraph is in control.
* Added a hardware test on the support page. It returns a JSON document with chip information and every DS18B20 sensor on the one-wire bus, with live temperature, assigned role (fridge or beer) and whether the configured sensors were found.
* Updated the support page with links, version and feature information, device log viewer and error code help.
* Added fan actuator support and status views.
* Added a cooling safety cooldown to protect the compressor between cooling cycles.
* Added task watchdog handling to help prevent display and UI stalls, including during WiFi reconnection.
* Changed the scheduled restart interval to 25 hours.
* Refined temperature control, sensor handling, and control metrics.
* Refactored the BrewPi PID library to use floating point math for easier maintenance.
* Updated the BrewPi PID library and added a simulator for comparing controller behavior against the original BrewPi implementation.
* Updated dependencies and LVGL deprecated methods, and added documentation built with Sphinx.
* (Bug) Fixed timer overflow in the BrewPi PID library.

0.8.0
=====

* New api to allow for remote control via brewlogger
* Added check to re-init temp control if sensors change
* (Bug) Fixed issue with setting mode when Ble sensor is used.
* (Bug) Added pins for s3 board that was missing
* (Bug) Enable bluetooth in s3 build

0.7.0
=====

* Refactored TFT UI so it can be run in simulator on my laptop
* Updated all dependecies to latest versions
* Added attributes to mdns broadcast for use in my other software
* (Bug) Fixing issue with beer button not hidden when ble tempsensor is not found or ble scanning is disabled
* (Bug) Fixed BLE sender to send tag for invalid value data is not available.

0.6.0
=====

* Adding BLE temperature sensor support for Beer sensor so that controller can use Gravitymon ble as base for target temperature

0.5.0
=====

* Update web UI based on my new common component library. Updated dependecies. Now using the full 16Mb on the ESP32 PRO. Upgrading from 0.3 is not possible on the PRO due to new flash layout.
