.. _compiling:

Building from source
####################

The firmware uses PlatformIO and the browser UI is built with Vite.

Firmware
========

Install PlatformIO Core or the PlatformIO IDE, clone the repository, then
build the environment for the target board:

.. code-block:: sh

   pio run -e chamber-controller-32pro
   pio run -e chamber-controller-s2
   pio run -e chamber-controller-s3

The build configuration embeds the prebuilt browser UI assets from ``html/``.
To update them, install the UI dependencies and build the app as described
below, then run the repository's UI copy script before building firmware.

Web UI and mock server
======================

Use Node.js and npm from the ``ui`` directory:

.. code-block:: sh

   cd ui
   npm install
   npm run mock

The mock API listens on ``http://localhost:3000``. Keep it running in its own
terminal. In a second terminal, start the Vite development server:

.. code-block:: sh

   cd ui
   npm run dev

Open the local Vite URL printed by the command. The UI expects the mock server
to be running to load simulated device status and configuration. Changes made
in the mock UI are for development only; they do not control physical
equipment. Build production assets with ``npm run build``.

The mock server returns representative status, configuration, and PID
structures and simulates mode changes and OneWire scans. It does not reproduce
all timing, sensor, hardware, or integration behavior of firmware.

Verification
============

Run the UI checks from ``ui`` with ``npm run test`` and ``npm run build``.
Before flashing a locally built firmware image, confirm its PlatformIO
environment, flash layout, and board pin definitions.
