#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
import collections
from robot_smach_states.util.startup import startup
from robot_skills.util import kdl_conversions
import PyKDL as kdl


class LearnOperator(smach.State):
    def __init__(self, robot, operator_timeout=20, ask_follow=True, learn_face=True, learn_person_timeout = 10.0):
        smach.State.__init__(self, outcomes=['follow', 'Failed'],
                             input_keys=['operator_learn_in'],
                             output_keys=['operator_learn_out'])
        self._robot = robot
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._learn_face = learn_face
        self._learn_person_timeout = learn_person_timeout
        self._operator_name = "operator_name"

    def execute(self, userdata):
        # operator = None                                             # local vs global variables?!?!
        start_time = rospy.Time.now()
        self._robot.head.look_at_standing_person()
        operator = userdata.operator_learn_in
        # import pdb; pdb.set_trace()
        while not operator:
            if self.preempt_requested():
                return 'Failed'

            if(rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return 'Failed'

            operator = self._robot.ed.get_closest_laser_entity(
                radius=0.5,
                center_point=kdl_conversions.VectorStamped(x=1.0, y=0, z=1,
                                                           frame_id="/%s/base_link" % self._robot.robot_name))
            rospy.loginfo("Operator: {op}".format(op=operator))
            if not operator:
                self._robot.speech.speak("Please stand in front of me")
            else:
                if self._learn_face:
                    self._robot.speech.speak("Please look at me while I learn to recognize you.",
                                             block=True)
                    self._robot.head.look_at_standing_person()
                    learn_person_start_time = rospy.Time.now()
                    num_detections = 0
                    while num_detections < 2: # 5:
                        if self._robot.perception.learn_person(self._operator_name):
                            self._robot.speech.speak("Succesfully detected you %i times" % (num_detections + 1))
                            num_detections += 1
                        elif (rospy.Time.now() - learn_person_start_time).to_sec() > self._learn_person_timeout:
                            self._robot.speech.speak("Please stand in front of me and look at me")
                            operator = None
                            break
        print "We have a new operator: %s" % operator.id
        self._robot.speech.speak("Gotcha! I will follow you!", block=False)
        self._robot.head.close()
        userdata.operator_learn_out = operator
        userdata.operator_id_out = operator.id
        return 'follow'

class Track(smach.State):  # Updates the breadcrumb path
    def __init__(self):
         smach.State.__init__(self,
                              outcomes=['track', 'no_track'],
                              input_keys=['buffer', 'operator'])      ## werkt dit zo met input keys?
         self.counter = 0
         self.period = 0.5          #fix this magic number

    def execute(self, userdata):
        # if self._operator_id:
        #     self._operator = self._robot.ed.get_entity(id=self._operator_id)
        #     if (rospy.Time.now().to_sec() - self._operator.last_update_time) > self._period:
        #         self._robot.speech.speak("Not so fast!")
        #
        #     # If the operator is still tracked, it is also the last_operator
        #     self._last_operator = self._operator
        #
        #     operator_pos = geometry_msgs.msg.PointStamped()
        #     operator_pos.header.stamp = rospy.get_rostime()
        #     operator_pos.header.frame_id = self._operator_id
        #     operator_pos.point.x = 0.0
        #     operator_pos.point.y = 0.0
        #     operator_pos.point.z = 0.0
        #     self._operator_pub.publish(operator_pos)
        #
        #     f = self._robot.base.get_location().frame
        #     self._operator_distance = self._last_operator.distance_to_2d(f.p)
        #
        #     return 'track'
        #
        # else:
        #      self._operator = None
        print userdata.operator
        # print userdata.operator_id

        if self.counter == 4:
            userdata.buffer.append(self.counter)
            print ("New breadcrumb added to buffer")
            self.counter = 0
            return 'no_track'
        self.counter += 1
        return 'track'


   # def __init__(self, robot, ask_follow=True, learn_face=True, operator_radius=1, lookat_radius=1.2, timeout=1.0,
   #               start_timeout=10, operator_timeout=20, distance_threshold=None, lost_timeout=60, lost_distance=0.8,
   #               operator_id_des=VariableDesignator(resolve_type=str), standing_still_timeout=20,
   #               operator_standing_still_timeout=3.0, replan=False):
   #
   #      smach.State.__init__(self, outcomes=["stopped", 'lost_operator', "no_operator"])
   #      self._robot = robot
   #      self._time_started = None
   #      self._operator = None
   #      self._operator_id = None
   #      self._operator_name = "operator"
   #      self._operator_radius = operator_radius
   #      self._lookat_radius = lookat_radius
   #      self._start_timeout = start_timeout
   #      self._breadcrumbs = []  # List of Entity's
   #      self._breadcrumb_distance = 0.1  # meters between dropped breadcrumbs
   #      self._operator_timeout = operator_timeout
   #      self._ask_follow = ask_follow
   #      self._learn_face = learn_face
   #      self._lost_timeout = lost_timeout
   #      self._lost_distance = lost_distance
   #      self._standing_still_timeout = standing_still_timeout
   #      self._operator_standing_still_timeout = operator_standing_still_timeout
   #      self._operator_id_des = operator_id_des
   #      self._operator_distance = None
   #      self._operator_pub = rospy.Publisher('/%s/follow_operator/operator_position' % robot.robot_name,
   #                                           geometry_msgs.msg.PointStamped, queue_size=10)
   #      self._plan_marker_pub = rospy.Publisher('/%s/global_planner/visualization/markers/global_plan' % robot.robot_name, Marker, queue_size=10)
   #      self._breadcrumb_pub = rospy.Publisher('/%s/follow_operator/breadcrumbs' % robot.robot_name, Marker,
   #                                             queue_size=10)
   #      self._face_pos_pub = rospy.Publisher('/%s/follow_operator/operator_detected_face' % robot.robot_name,
   #                                           geometry_msgs.msg.PointStamped, queue_size=10)
   #
   #      self._last_pose_stamped = None
   #      self._last_pose_stamped_time = None
   #      self._last_operator_fs = None
   #      self._replan_active = False
   #      self._last_operator = None
   #      self._replan_allowed = replan
   #      self._replan_timeout = 15 # seconds before another replan is allowed
   #      self._replan_time = rospy.Time.now() - rospy.Duration(self._replan_timeout)
   #      self._replan_attempts = 0
   #      self._max_replan_attempts = 3
   #      self._period = 0.5



class FollowBread(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread'],
                             input_keys=['buffer'])

    def execute(self, userdata):
        print list(userdata.buffer)
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
        smach.State.__init__(self, outcomes=['Failed', 'follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'follow'
        print "Oooh noooo, I give up."
        return 'Failed'


def setup_statemachine(robot):
    sm_top = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])
    sm_top.userdata.operator = None

    with sm_top:
        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(robot),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Failed': 'Failed'},
                               remapping={'operator_learn_in': 'operator', 'operator_learn_out': 'operator'})

        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Done': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'Failed': 'Failed',
                                            'follow': 'CON_FOLLOW'})

        sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize', 'keep_following'],
                                   default_outcome='keep_following',
                                   outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread',
                                                                 'TRACK': 'track'},
                                                'recover_operator': {'FOLLOWBREAD': 'no_follow_bread',
                                                                     'TRACK': 'no_track'}})

        sm_con.userdata.buffer = collections.deque([1])
        sm_con.userdata.operator = sm_top.userdata.operator

        with sm_con:
            smach.Concurrence.add('FOLLOWBREAD', FollowBread(), remapping={'buffer': 'buffer'})

            smach.Concurrence.add('TRACK', Track(), remapping={'buffer': 'buffer',
                                                               'operator': 'operator'})

        smach.StateMachine.add('CON_FOLLOW', sm_con,
                               transitions={'recover_operator': 'RECOVERY',
                                            'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'CON_FOLLOW'})

        return sm_top


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


### new idea::
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = collections.deque()
#
#     # def set_data(self, data):
#     #     # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#     #     # meerdere malen een punt toe voeg voordat ik weer uitlees
#     #     # Bij het 'appenden' kun je ook meteen een distance check doen.
#     #     with self._lock:
#     #         self._list = copy.deepcopy(data)
#     #
#     #         # Equivalent to
#     #         # self._lock.acquire()
#     #         # self._list = copy.deepcopy(data)
#     #         # self._lock.release()
#
#     def append(self, data):
#         # ToDo Josja
#         self._breadcrumb.append(data)
#
#
## note that all the buffervariables are popped and placed in a different variable which can be used for planning
#     def get_data(self):
#         result = []
#         while len(self._breadcrumb) > 0:
#             result.append(self._breadcrumb.pop())  # Pop or popleft?
#         return result


