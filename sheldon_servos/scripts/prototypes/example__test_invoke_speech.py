#! /usr/bin/env python

#
# Usage: test_invoke_speech.py <text_to_speak>
#

import argparse
import sys
import signal
import rospy
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

def signal_handler(signal, frame):
    if( None != client):
        rospy.loginfo("Ctrl-c, canceling speech");
        client.cancel_goal()
    sys.exit(0)

if __name__ == '__main__':
    client = None

    parser = argparse.ArgumentParser(description="Helper utility to invoke speech action")
    parser.add_argument("-s", "--speech", required=True, help="text of speech to speak", dest="speech")
    args = parser.parse_args()

    rospy.init_node('test_invoke_speech')
    signal.signal(signal.SIGINT, signal_handler);

    rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
    client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
    client.wait_for_server()

    rospy.loginfo("Invoke speech: %s", sys.argv[1])
    goal = audio_and_speech_common.msg.speechGoal(text_to_speak=args.speech)
    client.send_goal(goal)

    result = client.wait_for_result()
    rospy.loginfo("Behavior returned result: %d", result)
