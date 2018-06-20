import math

import numpy as np
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import LaserScan
from smach import StateMachine, State
from visualization_msgs.msg import MarkerArray, Marker

_ = tf2_geometry_msgs


class CustomFindCup(State):
    def __init__(self, robot, box_offset_x=1, box_size_y=1.5, box_size_x=1):
        State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['position'])
        self._visualization_publisher = rospy.Publisher('find_cup_visualization', MarkerArray, queue_size=1)

        self._robot = robot
        self._box_offset_x = box_offset_x
        self._box_size_y = box_size_y
        self._box_size_x = box_size_x

    def _extract_cup_pose(self, exclude_points=[], exclude_radius=0.1, cluster_max_distance=0.05,
                          cluster_min_points=10):
        def distance(p1, p2):
            return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

        def to_close_to_one_of_these_points(p, points, radius):
            for e_p in points:
                if distance((e_p, p)) < radius:
                    return True
            return False

        msg = rospy.wait_for_message("/amigo/torso_laser/scan", LaserScan)
        if msg is None:
            return None

        m_points = Marker()
        m_points.header = msg.header
        m_points.pose.orientation.w = 1
        m_points.ns = 'points'
        m_points.type = Marker.POINTS
        m_points.scale.x = m_points.scale.y = m_points.scale.z = 0.04
        m_points.color.r = m_points.color.a = 1.0

        clusters = []
        current_cluster = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r):
                continue

            x = r * math.cos(msg.angle_min + i * msg.angle_increment)
            y = r * math.sin(msg.angle_min + i * msg.angle_increment)

            # Skip if too close to exclude points
            if to_close_to_one_of_these_points((x, y), exclude_points, exclude_radius):
                continue

            # Skip if not within the box
            if x < self._box_offset_x - self._box_size_x / 2. or x > self._box_offset_x + self._box_size_x / 2. or y < - self._box_size_y / 2. or y > self._box_size_y / 2.:
                continue

            m_points.points.append(Point(x=x, y=y))
            if not current_cluster:
                current_cluster = [(x, y)]
            elif distance(current_cluster[-1], (x, y)) < cluster_max_distance:
                current_cluster.append((x, y))
            else:
                clusters.append(current_cluster)
                current_cluster = [(x, y)]
        if current_cluster:
            clusters.append(current_cluster)

        rospy.loginfo("Found %d clusters" % len(clusters))

        if not clusters:
            return None

        cluster_markers = []
        medians = []
        for i, cluster in enumerate(clusters):
            if len(cluster) < cluster_min_points:
                continue
            median_x = np.median([c[0] for c in cluster])
            median_y = np.median([c[1] for c in cluster])

            medians.append((median_x, median_y))

            m = Marker()
            m.id = i
            m.header = msg.header
            m.pose.orientation.w = 1
            m.pose.position.x = median_x
            m.pose.position.y = median_y
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.type = Marker.SPHERE
            m.ns = 'cluster'
            m.color.b = 1.0
            m.color.a = 1.0
            cluster_markers.append(m)

        # Sort cluster on closest
        rospy.loginfo("Medians %s", medians)
        candidate = min(medians, key=lambda x: x[0])
        rospy.loginfo("Candidate %s", candidate)

        m_candidate = Marker()
        m_candidate.header = msg.header
        m_candidate.pose.orientation.w = 1
        m_candidate.pose.position.x = candidate[0]
        m_candidate.pose.position.y = candidate[1]
        m_candidate.scale.x = m_candidate.scale.y = m_candidate.scale.z = 0.08
        m_candidate.type = Marker.SPHERE
        m_candidate.ns = 'candidate'
        m_candidate.color.g = m_candidate.color.r = m_candidate.color.a = 1.0

        m = Marker()
        m.header = msg.header
        m.pose.orientation.w = 1
        m.pose.position.x = self._box_offset_x
        m.scale.x = self._box_size_x
        m.scale.y = self._box_size_y
        m.scale.z = 0.1
        m.type = Marker.CUBE
        m.ns = 'box'
        m.color.g = 1.0
        m.color.a = 0.5
        self._visualization_publisher.publish(
            MarkerArray(markers=[m, m_points, m_candidate] + cluster_markers)
        )

        return PointStamped(
            header=msg.header,
            point=m_candidate.pose.position
        )

    def execute(self, ud):
        while True:
            rospy.loginfo("Exctracted cup pose: %s", self._extract_cup_pose())
        ud.position = PointStamped
        return 'succeeded'


class TestCustomFindCup(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            StateMachine.add("FIND_CUP",
                             CustomFindCup(robot),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_custom_find_cup')

    # robot = Amigo()
    # robot.ed.reset()
    # robot.leftArm.reset()
    # robot.torso.reset()

    sm = TestCustomFindCup(None)
    sm.execute()
