#!/usr/bin/python

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import frame_stamped
from hmi import TimeoutException
from geometry_msgs.msg import PointStamped
from people_msgs.msg import People

class WaitForCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted', 'rejected'])
        self._robot = robot
        self._caller_id = caller_id
        self._people_sub = rospy.Subscriber(robot.robot_name + '/persons', People, self.people_cb)
        self.rate = 10
        self.people_received = People()

    def execute(self, userdata=None):
        """ Does the actual work

        :param userdata:
        :return:
        """

        self._robot.head.reset()
        rospy.sleep(1)

        rospy.loginfo('waiting for waving person')
        waving_persons = []
        while not rospy.is_shutdown() and not waving_persons:
            rospy.sleep(1/self.rate)
            for person in self.people_received.people:
                if {'RWave', 'LWave'}.intersection(set(person.tags)):
                    waving_persons.append(person)

        if not waving_persons:
            return 'aborted'

        rospy.loginfo('waving persons: %s', waving_persons)
        if len(waving_persons) > 1:
            rospy.logwarn('using the first person')

        header = self.people_received.header
        point = waving_persons[0].position
        pose = frame_stamped(header.frame_id, point.x, point.y, 0.0)

        self._robot.speech.speak("I have seen a waving person, should I continue?")

        if self._confirm():
            rospy.loginfo('update customer position to %s', pose)
            self._robot.ed.update_entity(id=self._caller_id, frame_stamped=pose, type="waypoint")
            return 'succeeded'
        else:
            return 'rejected'

    def people_cb(self, persons):
        self.people_received = persons

    def _confirm(self):
        cgrammar = """
        C[True] -> amigo take the order
        C[False] -> amigo wait
        """
        self._robot.head.look_at_standing_person()
        for i in range(3):
            try:
                speech_result = self._robot.hmi.query(description="Should I get the order?",
                                                      grammar=cgrammar, target="C")
                return speech_result.semantics
            except TimeoutException:
                pass
        return False


class WaitForClickedCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted', 'rejected'])
        self._robot = robot
        self._caller_id = caller_id
        self._sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.rate = 10
        self._point = None

    def callback(self, point_stamped):
        rospy.loginfo("Recieved a point:\n{}".format(point_stamped))
        self._point = point_stamped

    def execute(self, userdata):
        self._robot.speech.speak("I'm waiting for a customer")
        rospy.loginfo("You can click in rviz")

        rate = rospy.Rate(self.rate)
        self._point = False
        while not rospy.is_shutdown() and not self._point:
            rate.sleep()

        if not self._point:
            return 'aborted'

        # TODO, get data from point into ED
        pose = frame_stamped("map", self._point.point.x, self._point.point.y, 0.0)
        self._robot.ed.update_entity(id=self._caller_id, frame_stamped=pose, type="waypoint")
        return 'succeeded'


if __name__ == '__main__':
    rospy.init_node('wait_for_customer')

    from robot_skills.amigo import Amigo
    robot = Amigo()
    robot.ed.reset()

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    with sm:
        smach.StateMachine.add('STORE_WAYPOINT',
                               WaitForCustomer(robot),
                               transitions={
                                    'succeeded' : 'done',
                                    'failed'  : 'done',
                                    'aborted' : 'done',
                                    'rejected' : 'done'})

    # states.startup(setup_statemachine, challenge_name="automatic_side_detection")
    sm.execute()

