#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class door(object):
    def __init__(self, W, id, x = 0, y = 0, th = 0, dooropen=False):
        self.id = id
        self.x = x
        self.y = y
        self.th = th
        self.dooropen = dooropen

        self.door = W.add_object("door-" + self.id, "robotics_testlabs.door",  self.x, self.y, 0.0, 0.0, 0.0, self.th )

    def toggleOpen(self):
        if self.dooropen:
            self.door.set_position(self.x, self.y, 0.0, 0.0, 0.0, self.th)
            self.dooropen = False
            print "Door " + self.id + " is closed"
        else:
            self.door.set_position(self.x, self.y,-3.0, 0.0, 0.0, self.th ) # TODO: make door swing open instead of move down by 3 meters
            self.dooropen = True
            print "Door " + self.id + " is opened"


if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    door1 = door(W, "1", 1.2, 3.8, 1.57)
    door2 = door(W, "2", 1.4, 3.3, 0.0 )
    door3 = door(W, "3",-0.5,-0.4, 1.57)

    while not rospy.is_shutdown():

        key = getKey()
        if key == '1':
            door1.toggleOpen()
        elif key == '2':
            door2.toggleOpen()
        elif key == '3':
            door3.toggleOpen()
        else:
            break