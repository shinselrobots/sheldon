#!/bin/bash          
#
# Script to activate environment and optionally delay the launch of a roslaunch file
# modified from script by Koen Lekkerkerker
# Use: ./activated_launch.sh [number of seconds to delay] [rospkg] [roslaunch file]
#

echo "start wait for $1 seconds..."
sleep $1
echo "end wait for $1 seconds"
echo "Activating environment for tensorflow..."
source ~/sdk/tensorflow/bin/activate

# Get other arguments
shift
    echo "now running 'roslaunch $@'"
roslaunch $@

