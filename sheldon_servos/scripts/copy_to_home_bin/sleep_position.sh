#!/usr/bin/env sh
# put robot in sleep mode

/bin/echo "Entering Sleep Mode..."

rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "SLEEP" "a" "b"


