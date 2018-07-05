#! /usr/bin/env python

import rospy
import actionlib
import behavior_common.msg
#from std_msgs.msg import Float64

# THIS NULL BEHAVIOR DOES NOTHING!  
# Used for testing, so robot will not be moving (unlike Idle behavior)

class BehaviorAction(object):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Python behavior service' % (self._action_name))

    def execute_cb(self, goal):
        r = rospy.Rate(1)

        # Start Null Behavior
        while True:
            if self._as.is_preempt_requested():
                break 
            r.sleep();

        # Null behavior runs forever until interrupted, so this behavior
        # can only get prempted/cancelled.
        self._as.set_preempted();
        
if __name__ == '__main__':
    rospy.init_node('null_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
