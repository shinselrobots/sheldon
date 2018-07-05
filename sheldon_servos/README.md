# sheldon_servos

Dynamixel setup:
Download Robotis RoboPlus (windows app) from Robotis website
In the RoboPlus Dynamixel Wizard, set:

SHOULDER SERVOS:
All 4 MX106 servos are configured for multiple turn, 2x
Right Arm:  set "drive mode" of servo #16 to master, #18 to slave, both *REVERSED*
Left Arm: set "drive mode" of servo #17 to master, #19 to slave (not reversed)
Connect master and slave servos with special robotis slave cable
Set offset of both to 1024 (1/4 turn) to avoid bug where startup forgets where zero is!


NOTE:  If arms seem in the wrong positions (not homing to correct spot), check tightness
of screws in shoulder gear!
