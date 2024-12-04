![download](https://img.shields.io/github/downloads/mp-se/chamber-controller/total) 
![release](https://img.shields.io/github/v/release/mp-se/chamber-controller?label=latest%20release)
![issues](https://img.shields.io/github/issues/mp-se/chamber-controller)
![pr](https://img.shields.io/github/issues-pr/mp-se/chamber-controller)

# Overview

This is a project that I created to test out a PID controller for another project that I'm working on. It's based on the BrewPI software where I have extracted the PID controller into a library that can be reused. That library can be found here; https://github.com/mp-se/brewpi-pid-library to honour that licence agreement. 

This is still work in progress but so far I have been running this PID controller for a couple of weeks without issues.

# Features

* Control heat and cooling outputs. 
* Supports 2 temperature senors for beer and chamber.
* Control either sensor target.
* Sending data to influxdb v2 for analysis of PID algorithm and fine tuning.
* Web interface for easy configuration.
* Firmware update and Serial Logging via web interface. 

# Hardware

Currently I'm using the following development board for running my instance but the frameworks would support many more boards with some new targets. 

* Lolin ESP32 PRO with Lolin TFT

# Flashing

Currently I use VSCode and PlatformIO to build and flash the device. Pre-built binaries are available and can be flashed using esptool.  

Another option is to use python and esptool for flashing. Run the commands from the root directory for this project.

- Install python3 from python.org
- run> pip install esptool
- run> python3 flash.py target port

Example: python3 flash.py esp32 COM10

# Software Setup

All the configuration is done using a web interface running on the device but after flashing there is a need to setup the wifi support. After installation the device will create an SSID called Chamber with the password 'password'. Join this network and then navigate to http://192.168.4.1 to open up the user interface. Under WIFI you can scan for existing networks, select the one you want and enter the SSID. Once the wifi is settings is saved you can reset the device and it should connect to the network. If you have an display the IP adress is shown on the bottom of the screen.

