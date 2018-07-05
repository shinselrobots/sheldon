

    To make Feather boards work:
    https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules
    “The rules also fix an issue with ModemManager hanging on to /dev/ttyACM devices”

    1. wget https://github.com/adafruit/Trinket_Arduino_Linux/raw/master/99-adafruit-boards.rules
    2. sudo cp 99-adafruit-boards.rules /etc/udev/rules.d/
    3. Reboot

    Since feather does not have serial number, udev rules have been setup by physical USB port


