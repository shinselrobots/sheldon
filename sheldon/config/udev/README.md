# UDEV Rules for Sheldon Robot

## Sheldon has lots of devices, these make sure everything gets mapped correctly
    * To install:  ./create_udev_rules.sh


## Normal Sheldon Config should include the following:
    ls -la /dev | grep "\->"
    base_arduino        -> ttyACMx
    base_odom_arduino   -> ttyACMx
    dynamixel           -> ttyUSBx
    head_arduino        -> ttyACMx
    left_arm_arduino    -> ttyACMx
    right_arm_arduino   -> ttyACMx
    rplidar             -> ttyUSBx
    sabertooth          -> ttyACMx


## Adafruit Feather boards
    Since feather does not have serial number, udev rules have been setup by physical USB port
    See UDEV rules: ~/catkin_robot/src/sheldon/sheldon/config/udev

    To get these values, use:
    udevadm info -q all -n /dev/ttyACM0 | grep DEVPATH
                                                                                  >   use this   <
    ODOM:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.2/.              > 1-3.2:1.0.   <  /tty/ttyACM0
    ARML:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.1/  > 1-2.4.1:1.0  <  /tty/ttyACM2     
    ARMR:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.2/  > 1-2.4.2:1.0  <  /tty/ttyACM4

## In case of problems:
    https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules
    “The rules also fix an issue with ModemManager hanging on to /dev/ttyACM devices”

    1. wget https://github.com/adafruit/Trinket_Arduino_Linux/raw/master/99-adafruit-boards.rules
    2. sudo cp 99-adafruit-boards.rules /etc/udev/rules.d/
    3. Reboot


