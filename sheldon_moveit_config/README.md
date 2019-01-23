# Sheldon Moveit Config

- These config files define sheldon to the move-it package, to enable arm movement planning for picking up objects.

- Configuration info is mostly stored in ./config/sheldon_robot.srdf
  in particular, see "GROUP STATES" for pre-defined arm poses that can be used

## References:
- "Ros By Example", by R. Patrick Goebel
- http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html

## Install
sudo apt-get install ros-kinetic-moveit

- work around for issue https://github.com/ros-planning/moveit/issues/86 :
sudo dpkg --remove --force-depends python-pyassimp
sudo -H pip install pyassimp


## Testing
view the README.md in ./tests


