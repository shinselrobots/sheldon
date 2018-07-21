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

## Arduino
To support AdaFruit boards in Arduine IDE:
    * Open Arduino IDE
    * Preferences -> Additional Boards Manager URLs:
    * Add this: 
        https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
    * Optional:  See this list for more board support options: 
        https://github.com/arduino/Arduino/wiki/Unofficial-list-of-3rd-party-boards-support-urls
    * Install board using the Arduino Board Manager.  More info here:
        https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51/setup
    * For Feather M4 Express (used in Sheldon Arms), install BOTH of these:
        * "Arduino SAMD Boards by Arduino"
        * "Adafruit SAMD Boards by Adafruit"
    * For Feather 32u4 (old), install 
        * "Adafruit AVR Boards by Adafruit"

## Adafruit Feather boards
    Since feather does not have serial number, udev rules have been setup by physical USB port
    See UDEV rules: ~/catkin_robot/src/sheldon/sheldon/config/udev

    To get these values, use:
    udevadm info -q all -n /dev/ttyACM0 | grep DEVPATH
                                                                                  >   use this   <
    ODOM:   DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.2/.              > 1-3.2:1.0.   <  /tty/ttyACM0
    ARM L:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.1/  > 1-2.4.1:1.0  <  /tty/ttyACM2     
    ARM R:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.2/  > 1-2.4.2:1.0  <  /tty/ttyACM4

    ARM R:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.2/  > 1-2.4.2:1.0  <   /tty/ttyACM0


## In case of problems:
    https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules
    “The rules also fix an issue with ModemManager hanging on to /dev/ttyACM devices”

    1. wget https://github.com/adafruit/Trinket_Arduino_Linux/raw/master/99-adafruit-boards.rules
    2. sudo cp 99-adafruit-boards.rules /etc/udev/rules.d/
    3. Reboot


