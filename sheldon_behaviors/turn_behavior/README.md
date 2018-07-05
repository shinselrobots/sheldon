# turn_behavior
To test (example commands):
  - rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "TURN" "'90'" "'1.5'"
  - rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "TURN" "'-45'" "'1.5'"


# About Turn Speed:
  - Turn Speed range is 1.0 - 3.0 Meters/Second
  - (less than 1.0 will stall out)
  - This speed range is used for all modules (Including Wheel Control)
  -   (except from Wheel_Control to Sabertooth, where speeds are 0.0 - 1.0)

