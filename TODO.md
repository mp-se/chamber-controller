# TODO

- Add gravitymon and pressuremon as external temperature sensors
- Flashing 0.4 requires full flashing since flash layout has changed for esp32pro

"/Users/magnus.persson6/.platformio/penv/bin/python" "/Users/magnus.persson6/.platformio/packages/tool-esptoolpy/esptool.py" --chip esp32 --port "/dev/cu.usbserial-10" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 16MB 0x1000 /Users/magnus.persson6/dev/chamber-controller/.pio/build/chamber-controller-32pro/bootloader.bin 0x8000 /Users/magnus.persson6/dev/chamber-controller/.pio/build/chamber-controller-32pro/partitions.bin 0xe000 /Users/magnus.persson6/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin 0x10000 .pio/build/chamber-controller-32pro/firmware.bin
esptool.py v4.5.1