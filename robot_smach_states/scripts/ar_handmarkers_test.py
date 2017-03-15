#!/usr/bin/python

"""This script can be used to gather data to see the offset between the
transforms (tf) from to the hand from the joint angles and the AR-
marker position.

In short, amigo first brings its hands in a grasping position, then
looks at one of the hands and prints the tf from that hand relative to
the marker on that hand."""

from robot_skills.amigo import Amigo
import rospy
import tf2_ros

if __name__ == "__main__":
    rospy.init_node("hands_ARmarkers_test", anonymous=True)
    amigo = Amigo()

    # Position the arms in grasping position and look at left hand
    rospy.loginfo('Sending left arm goal')
    amigo.leftArm._send_joint_trajectory([[0,0.5,0.3,1.2,0.15,0.15,-0.2]])

    rospy.loginfo('Sending right arm goal')
    amigo.rightArm._send_joint_trajectory([[0,0.5,0.3,1.2,0.15,0.15,-0.2]])

    rospy.loginfo('Sending head goal')
    amigo.head.look_at_hand("right")
    # amigo.head.look_at_hand("left")

    # Print transform
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo(tfBuffer.lookup_transform("amigo/hand_right", 
                                                    "ar_marker_3", 
                                                    rospy.Time(), 
                                                    rospy.rostime.Duration(5)))
            # rospy.loginfo(tfBuffer.lookup_transform("amigo/hand_left", 
            #                                         "ar_marker_6", 
            #                                         rospy.Time(), 
            #                                         rospy.rostime.Duration(5)))
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
