# sheldon_servos

## Dynamixel setup:
Download Robotis RoboPlus (windows app) from Robotis website
In the RoboPlus Dynamixel Wizard, set:

SHOULDER SERVOS:
All 4 MX106 Shoulder Servos are configured for multiple turn, 2x
Right Arm:  set "drive mode" of servo #18 to master, #20 to slave, 
            both *REVERSED* in .../sheldon_servos/config/dynamixel_params.yaml
Left Arm: set "drive mode" of servo #19 to master, #21 to slave (not reversed)
Connect master and slave servos with special robotis slave cable
Set offset of both to 1024 (1/4 turn) to avoid bug where startup forgets where zero is!


NOTE:  If arms seem in the wrong positions (not homing to correct spot), check tightness
of screws in shoulder gear!

# This package is setup for python scripts per ROS standard.
  - launchable python scripts are in ./scripts
  - shared modules for export are in ./src/sheldon_servos
  - see https://git-amd.tuebingen.mpg.de/aherzog/hinvdyn_example_workspace/blob/master/src/catkin/third_party/catkin/doc/howto/format2/installing_python.rst
