![download](https://img.shields.io/github/downloads/mp-se/chamber-controller/total) 
![release](https://img.shields.io/github/v/release/mp-se/chamber-controller?label=latest%20release)
![issues](https://img.shields.io/github/issues/mp-se/chamber-controller)
![pr](https://img.shields.io/github/issues-pr/mp-se/chamber-controller)

# Overview

This is a project that I created to test out a PID controller for another project that I'm working on. It's based on the BrewPI software where I have extracted the PID controller into a library that can be reused. That library can be found here; https://github.com/mp-se/brewpi-pid-library to honour that licence agreement. 

This is still work in progress but so far I have been running this PID controller for a couple of weeks without issues.

# Features

* Control heating and cooling output
* Supports 2 temperature sensors for beer and chamber.
* Control on either sensor target.
* Sending data to influxdb v2 for analysis of PID algorithm and fine tuning.
* Sending temperature data over bluetooth for use in my other projects
* Web interface for easy configuration.
* Firmware update and Serial Logging via web interface.

# Hardware

Currently I'm using the following development board for running my instance but the frameworks would support many more boards with some new targets. 

* Lolin ESP32 PRO with Lolin TFT
* Lolin ESP32s2 mini without TFT
* Lolin ESP32s3 mini without TFT

# Flashing

Flashing is done using my web flasher available at https://gravitymon.com/flasher/index.html.

Its not possible to upgrade from versions older than 0.3 to 0.4+ for the esp32pro board, I had to change the flash layout in order to accomodate larger binaries which is required to fit the bluetooth stack and TFT libraries.

# Software & Wifi Setup

All the configuration is done using a web interface running on the device but after flashing there is a need to setup the wifi support. After installation the device will create an SSID called Chamber with the password 'password'. Join this network and then navigate to http://192.168.4.1 to open up the user interface. Under WIFI you can scan for existing networks, select the one you want and enter the SSID. Once the wifi is settings is saved you can reset the device and it should connect to the network. If you have an display the IP adress is shown on the bottom of the screen.

