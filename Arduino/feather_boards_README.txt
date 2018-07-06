

    To make Feather boards work:
    https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules
    “The rules also fix an issue with ModemManager hanging on to /dev/ttyACM devices”

    1. wget https://github.com/adafruit/Trinket_Arduino_Linux/raw/master/99-adafruit-boards.rules
    2. sudo cp 99-adafruit-boards.rules /etc/udev/rules.d/
    3. Reboot

    Since feather does not have serial number, udev rules have been setup by physical USB port
    See UDEV rules: ~/catkin_robot/src/sheldon/sheldon/config/udev

    ODOM:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.2/.              > 1-3.2:1.0.   <  /tty/ttyACM0
    ARM1:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.4/     1-1.4.1/  > 1-1.4.1:1.0. <  /tty/ttyACM4
    ARM2:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.4/.    1-1.4.2/  > 1-1.4.2:1.0. <  /tty/ttyACM5

