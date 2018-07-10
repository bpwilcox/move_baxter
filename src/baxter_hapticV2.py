import time
import rospy
import baxter_interface
from math import sin
import numpy as np

from transforms3d.quaternions import (
    mat2quat,
    quat2mat
    )
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import (
    Header,
    String,
    Float32MultiArray
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from sensor_msgs.msg import JointState

from baxter_interface import CHECK_VERSION
baxter_transform = np.asarray([
                        [0,0,1],
                        [1,0,0],
                        [0,1,0],
                        ])
class haptic_pos:
    def __init__(self):
        self.hd_vel = np.zeros((3,1))
        self.hd_ang_vel = np.zeros((3,1))
        self.hd_transform = np.eye(4)
        self.hd_position = np.zeros((3,1))
        self.baxter_transform = np.asarray([
                                [0,0,1,0],
                                [1,0,0,0],
                                [0,1,0,0],
                                [0,0,0,1]
                                ])
        self.hd_button1 = False
        self.hd_button2 = False
        rospy.Subscriber('pose_msg',Float32MultiArray,self.callback)
    def callback(self,data_stream):
        self.hd_transform = np.reshape(data_stream.data[0:16],(4,4),order='F')
        self.hd_vel = np.asarray(data_stream.data[16:19])
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])
        if data_stream.data[22]==1:
            self.hd_button1 = True
        else:
            self.hd_button1 = False
        if data_stream.data[23]==1:
            self.hd_button2 = True
        else:
            self.hd_button2 = False

def R_offset(hd_transform_0,b_transform_0):
    return np.matmul(hd_transform_0.T,b_transform_0)

def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)

def main():
    log_joint_angles = []
    log_pos = []
    log_haptic = []
    rospy.init_node('Control_baxter')
    phantom = haptic_pos()

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Limb initialization
    limb = baxter_interface.Limb('right')
    limb.move_to_neutral()

    #IK Service
    ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')


    #Communication rate - 1kHz
    rate = rospy.Rate(50)
    def reset_baxter():
        #np.save('log_joint_angles.npy',np.asarray(log_joint_angles))
        #np.save('log_pos.npy',np.asarray(log_pos))
        #np.save('log_haptic.npy',np.asarray(log_haptic))
        limb.move_to_neutral()
        rs.disable()

    alpha = 1
    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x*alpha

    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])

    R_off = R_offset(hd_transform_0,b_transform_0)

    rospy.on_shutdown(reset_baxter)

    while not rospy.is_shutdown():
        old_joint_angles = limb.joint_angles()
        cur_pos = limb.endpoint_pose()['position']
        des_ori = np.matmul(np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3]),R_off)
        ori = mat2quat(des_ori)

        cur_ori = limb.endpoint_pose()['orientation']
        des_ori_command  = baxter_interface.Limb.Quaternion(w=ori[0],x=ori[1],y=ori[2],z=ori[3])
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
        des_x = alpha*hd_x + x_off

        if not phantom.hd_button1:
            des_pos = Point(des_x[0],des_x[1],des_x[2])
            pose= PoseStamped(header=hdr,pose=Pose(position= des_pos,orientation=des_ori_command))
            ikreq.pose_stamp.append(pose)
            resp = iksvc(ikreq)

            if (resp.isValid[0]):
                print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                #print limb_joints
            else:
                print("INVALID POSE - No Valid Joint Solution Found.")
                des_pos = cur_pos
                limb_joints = old_joint_angles

            limb.set_joint_positions(limb_joints,raw=True)
            ikreq.pose_stamp.pop()
        else:
            while phantom.hd_button1:
                pass
            hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off = R_offset(hd_transform_0,b_transform_0)
            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            x_off = b_x - hd_x*alpha

        rate.sleep()

    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
