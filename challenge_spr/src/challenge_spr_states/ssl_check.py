#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
from robot_smach_states.util.startup import startup

class SslCheck(smach.State):
    def __init__(self, robot, timeout=5):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.robot.ssl._sub.get_num_connections() > 0:
                rospy.loginfo("SSL connection established")
                return "succeeded"
            elif (rospy.Time.now() - start_time).to_sec() > self.timeout
                break
            rospy.sleep(0.5)  #TODO: correct api?
        rospy.logerr("No SSL connection found")
        return "failed"
