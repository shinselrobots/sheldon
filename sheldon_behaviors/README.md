# sheldon_behaviors
# lots of behaviors here.  See robot_behavior for underlying framework

# Installation Instructions / Prerequisites:
  * Some behaviors play a wav file locally.
  -  For python we use playsound:
  -  sudo -H pip install playsound

# Testing Behaviors:
  - Invoking from command line (note extra quotes for numeric parameters)
  - rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "WAVE" "a" "b"
  - rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "TURN" "'--90'" "'0.4'"

# Debugging Behaviors:
  - Behaviors can be manually invoked.  
    - Start the robot stack as usual (eg., roslaunch sheldon robot.launch)
    - In a separate shell invoke behavior as above.
  - Notes:
    - You will notice some errors as the new version replaces the old, these can be ignored while debugging
    - After debugging the origional behavior will not invoke, so you need to restart the robot stack
