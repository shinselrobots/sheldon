# Installing Arduino on Linxu for Sheldon

1. Make sure you configure Arduino as follows:

File-> Preferences -> Sketchbook location: ~/catkin_robot/src/sheldon/Arduino
  (if you do this, it will automatically include libraries checked in to git)

2.  Check for and Install any missing libraries (compile will show any missing)

    - Tools --> Manage Libraries
    - Adafruit Circuit Playground
    - Adafruit BN0055
    - Adafruit NeoPixel
    - Adafruit Unified Sensor
    - Encoder by Paul Stoffregen

3.  Install RosSerial
    sudo apt-get update
    sudo apt-get install ros-kinetic-rosserial-arduino
    sudo apt-get install ros-kinetic-rosserial

4.  Select the board type depending which Arduino board you are modifying

5.  Set COM Port to port where Arduino is connected



6. See sheldon_base_arduino_readme.md for setting up Android phone connection


7. To make Feather boards work, see 
   ~/catkin_robot/src/sheldon/sheldon/config/udev/README.md

8. Adding custom messages: you need to rerun:
   cd ~/catkin_robot/src/sheldon/Arduino/libraries
   rm -r ros_lib
   rosrun rosserial_arduino make_libraries.py .
   restart Arduino.

