import time
import rospy
import baxter_interface
from math import sin

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import (

    Header,
    String
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

class haptic_pos:
    def __init__(self):
        self.cur_vel = Point(0,0,0)
        self.alpha = 0.01
        rospy.Subscriber('pose_msg',Point,self.callback)

    def callback(self,data):
        self.cur_vel.x = data.z * self.alpha
        self.cur_vel.y = data.x * self.alpha
        self.cur_vel.z = data.y * self.alpha

def main():
        rospy.init_node('Control_baxter')
        phantom = haptic_pos()
        kin = baxter_kinematics('right')

        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        print("Enabling robot... ")
        rs.enable()

        #Ros initialization
        limb = baxter_interface.Limb('right')
        limb_names = limb.joint_names()
        limb.move_to_neutral()
        def reset_baxter():
            limb.move_to_neutral()
            rs.disable()
        x = 0
        rate = rospy.Rate(10)
        rospy.on_shutdown(reset_baxter)
        while not rospy.is_shutdown():
            old_joint_angles = limb.joint_angles()
            #cur_pos = limb.endpoint_pose()['position']
            cur_pos = kin.forward_position_kinematics()
            cur_ori = limb.endpoint_pose()['orientation']

            #Get increment from haptic device
            delta_pos = baxter_interface.Limb.Point(phantom.cur_vel.x,phantom.cur_vel.y,phantom.cur_vel.z)

            #Query Baxter's current position
            #new_pos = [cur_pos.x+delta_pos.x,cur_pos.y+delta_pos.y,cur_pos.z+delta_pos.z]
            x += 0.01
            #new_pos = [cur_pos.x+0.001*sin(2*3.14*x),cur_pos.y,cur_pos.z]
            new_pos = [cur_pos[0]+0.001*sin(2*3.14*x),cur_pos[1],cur_pos[2]]
            print(new_pos)
            #IK pose
            new_joint_angles = kin.inverse_kinematics(new_pos)
            if new_joint_angles is not None:
                limb_joints = dict(zip(limb_names,np.deg2rad(new_joint_angles)))
                # Baxter joint control
            else:
                print('No solution')
                limb_joints = old_joint_angles
            limb.set_joint_positions(limb_joints,raw=True)
            rate.sleep()

        limb.move_to_neutral()
        rs.disable()

if __name__=='__main__':
    main()
