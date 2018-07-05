#!/bin/bash

while [ true ]; do
  rostopic pub -1 /head_pan_controller/command std_msgs/Float64 -- 0.0 
  rostopic pub -1 /head_tilt_controller/command std_msgs/Float64 -- 0.0 
  rostopic pub -1 /head_tilt_controller/command std_msgs/Float64 -- 1.0 
  rostopic pub -1 /head_pan_controller/command std_msgs/Float64 -- 1.0 
  rostopic pub -1 /head_pan_controller/command std_msgs/Float64 -- -1.0 
  rostopic pub -1 /head_tilt_controller/command std_msgs/Float64 -- -1.0 
  rostopic pub -1 /head_pan_controller/command std_msgs/Float64 -- 1.0 
done
