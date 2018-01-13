#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
from robot_smach_states.util.startup import startup


class LearnOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['follow'])

    def execute(self, userdata=None):
        num_detections = 0
        while num_detections < 5:
            var = raw_input("Did I successfully detect you? Yes/No: ")
            if var == "Yes":
                num_detections += 1
        print ("Detected operator successfully 5 times, start following...")
        # The operator should be added to the breadcrumb list here.
        return 'follow'


class Track(smach.State):  # Updates the breadcrumb path
    def __init__(self):
        smach.State.__init__(self, outcomes=['track', 'no_track'])
        self.counter = 0

    def execute(self, userdata=None):
        if self.counter == 10:
            return 'no_track'
        self.counter += 1
        return 'track'


class FollowBread(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['follow_bread', 'no_follow_bread'])
        self.counter = 0

    def execute(self, userdata=None):
        if self.counter == 5:
            return 'no_follow_bread'
        self.counter += 1
        return 'follow_bread'


class AskFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['follow', 'Done'])
        self.counter2 = 0

    def execute(self, userdata=None):
        var = raw_input("Am I done following you? Yes/No: ")
        if var == "No":
            return 'follow'
        print "Okidoki, we reached the final destination."
        return 'Done'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed', 'follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'follow'
        print "Oooh noooo, I give up."
        return 'Failed'


def setup_statemachine(robot):
    sm_top = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])

    with sm_top:

        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(),
                               transitions={'follow': 'CON_FOLLOW'})

        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Done': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'failed': 'Failed',
                                            'follow': 'CON_FOLLOW'})

        sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize'],
                                   outcome_map={'ask_finalize':
                                                    {'FOLLOWBREAD': 'no_follow_bread',
                                                     'TRACK': 'track'}
                                                'recover_operator':
                                                    {'FOLLOWBREAD': 'no_follow_bread',
                                                     'TRACK': 'no_track'}})


        with sm_con:
            smach.Concurrence.add('FOLLOWBREAD', FollowBread())
            smach.Concurrence.add('TRACK', Track())

        smach.StateMachine.add('Con_Follow', sm_con,
                               transitions={'recover_operator': 'RECOVERY',
                                            'ask_finalize': 'ASK_FINALIZE'})

        return sm_top


 # smach.StateMachine.add('TRACK', Track(),
 #                               transitions={'ask_finalize': 'ASK_FINALIZE',
 #                                            'keep_following': 'TRACK',
 #                                            'recover_operator': 'RECOVERY'})


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)


# import copy, threading

# Sample code Janno
# This class should rather be named Buffer instead of BreadCrumb: it is only used to pass data. The 'breadcrumb' is
# the list that is kept in the execute function of the Follow state (hence NOT a member of the class)
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = []
#         self._lock = threading.Lock()
#
#     def set_data(self, data):
#         # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#         # meerdere malen een punt toe voeg voordat ik weer uitlees
#         # Bij het 'appenden' kun je ook meteen een distance check doen.
#         with self._lock:
#             self._list = copy.deepcopy(data)
#
#             # Equivalent to
#             # self._lock.acquire()
#             # self._list = copy.deepcopy(data)
#             # self._lock.release()
#
#     def append(self):
#         # ToDo Josja
#         pass
#
#     def get_data(self):
#         with self._lock:
#             result = self._breadcrumb
#             self._breadcrumb = []
#         return result
#
#
# class Track(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self.robot = robot
#         self._breadcrumb = breadcrumb
#
#         # Possible new way:
#         # self._tracking_sub = rospy.Subscriber("bla", bla_msgs.Blaat, self._tracking_callback)
#
#     def execute(self):
#         while True:
#             operator = self.robot.ed.get_person()
#             self._breadcrumb.append(operator)
#
#             # self._breadcrumb.append(self._new_data)
#             # self._new_data = []
#
#             # def _tracking_callback(self, msg):
#             #     self._new_data = msg....
#             #     self._breadcrumb.append(msg.data)
#
#
# class Follow(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self._breadcrumb = breadcrumb
#
#         self._lookat_radius = 1.0  # Something like that
#
#         def execute(self):
#             breadcrumb = []  # Create an empty list at the top of our execute hook: we don't want any state remaining
#
#             while True:
#                 new_list
#                 self._breadcrumb.get_data()
#                 breadcrumb = breadcrumb + new_list
#
#                 # Do smart stuff with breadcrumb
#
#                 # Pitfall: breadcrumb does still contain data when entering this hook the second time. How do we solve this?
#
#                 # ToDo: remove points that have been visited by the robot
#
#
# class FollowMachine(smach.ConcurrentStateMachine):
#     breadcrumb = BreadCrumb()
#
#     with sm:
#         smach.StateMachine.add("track", Track(breadcrumb=breadcrumb))
#         smach.StateMachine.add("Follow", Follow(breadcrumb=breadcrumb))
#
#
#
