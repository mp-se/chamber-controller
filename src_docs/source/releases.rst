.. _releases:

Release notes
#############

0.9.0
=====

* Added BrewGraph integration, including configuration, chamber reporting, and integration status in the web interface.
* Added fan actuator support and status views.
* Added a cooling safety cooldown to protect the compressor between cooling cycles.
* Added task watchdog handling to help prevent display and UI stalls.
* Refined temperature control, sensor handling, and control metrics.
* Refactored the BrewPi PID library to use floating point math for easier maintenance .
* Updated the BrewPi PID library and added a simulator for comparing controller behavior.

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
