# stop_behavior

# Interrupts whatever behavior is running and stops wheels

# To test:
  rostopic pub -1 /behavior/cmd behavior_common/CommandState "STOP" "a" "b"
