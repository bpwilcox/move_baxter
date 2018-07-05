import time
import rospy
import baxter_interface
from math import sin
import numpy as np
import sys
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import (

    Header,
    String,
    Empty
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from baxter_interface import CHECK_VERSION

import moveit_commander
import moveit_msgs.msg

class haptic_pos:
    def __init__(self):
        self.cur_vel = Point(0,0,0)
        self.alpha = 0.005
        rospy.Subscriber('pose_msg',Point,self.callback)

    def callback(self,data):
        self.cur_vel.x = data.z * self.alpha
        self.cur_vel.y = data.x * self.alpha
        self.cur_vel.z = data.y * self.alpha

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Baxter_moveit',anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
if __name__=='__main__':

    main()
