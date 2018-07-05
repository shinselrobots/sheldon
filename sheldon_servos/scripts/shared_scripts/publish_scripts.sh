#!/bin/bash
#
# copy scripts to the correct location
#

rm ~/catkin_robot/devel/lib/python2.7/dist-packages/sheldon_servos/*.pyc
cp *.py ~/catkin_robot/devel/lib/python2.7/dist-packages/sheldon_servos
echo "~/catkin_robot/devel/lib/python2.7/dist-packages/sheldon_servos:"
ls -la ~/catkin_robot/devel/lib/python2.7/dist-packages/sheldon_servos

