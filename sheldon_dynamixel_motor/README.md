dynamixel_motor
===============

ROS stack for interfacing with Robotis Dynamixel line of servo motors.

# NOTE - MODIFIED FOR SHELDON
This project was cloned from arebgun/dynamixel_motor and modified for the Sheldon Robot 
to support geared servos for the shoulders (2:1 reduction gear ratio).

The change is active whenever the dynamixel servo "resolution divider" register is set to > 1
see the robotis servo manual for the servo for details.

Modifications:  

  dynamixel_driver/dynamixel_const.py:
    - added DXL_RESOLUTION_DIVIDER register for reading the gear ratio (set in the servo)

  dynamixel_driver/src/dynamixel_driver/dynamixel_io.py
      - added get_resolution_divider() function

  dynamixel_driver/src/dynamixel_driver/dynamixel_serial_proxy.py
      - use get_resolution_divider to compensate for external gear in related calculations
      - NOTE: get_resolution_divider effects speed and torque calculations, 
      -       but not position, which is handled internally by the servo  

