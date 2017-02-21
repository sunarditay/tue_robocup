#!/usr/bin/python

"""This script can be used to gather data to see the offset between the
transforms (tf) from to the hand from the joint angles and the AR-
marker position.

In short, amigo first brings its hands in a grasping position, then
looks at one of the hands and publishes the tf from that hand to the
marker on that hand."""

from robot_skills.amigo import Amigo
import rospy

if __name__ == "__main__":
    rospy.init_node("hands_ARmarkers_test", anonymous=True)
    amigo = Amigo()

    # Create topics for the left and right hand
    # left_tf  = rospy.Publisher('lefthand_error',  std_msgs.msg.String, queue_size=10)
    # right_tf = rospy.Publisher('righthand_error', std_msgs.msg.String, queue_size=10)

    # Position the arms in grasping position and look at left hand
    rospy.loginfo('Sending left arm goal')
    amigo.leftArm._send_joint_trajectory([[0,0.5,0.3,1.2,-0.15,-0.1,-0.2]])

    rospy.loginfo('Sending right arm goal')
    amigo.rightArm._send_joint_trajectory([[0,0.5,0.3,1.2,-0.15,-0.1,-0.2]])

    rospy.loginfo('Sending head goal')
    amigo.head.look_at_hand("left")

    # Publish tf
    # left_tf.publish()
    # right_tf.publish()