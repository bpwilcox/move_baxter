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
        #self.hd_transform = np.matmul(self.baxter_transform,self.hd_transform)
        self.hd_vel = np.asarray(data_stream.data[16:19])
        #self.hd_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_vel)
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])
        #self.hd_ang_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_ang_vel)
        #self.hd_position = np.asarray(data_stream.data[22:25])
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
    x = 0
    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])

    R_off = R_offset(hd_transform_0,b_transform_0)

    rospy.on_shutdown(reset_baxter)

    while not rospy.is_shutdown():
        old_joint_angles = limb.joint_angles()
        cur_pos = limb.endpoint_pose()['position']
        hd_ori = np.matmul(np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3]),R_off)

        ori = mat2quat(hd_ori)
        cur_ori = limb.endpoint_pose()['orientation']
        des_ori  = baxter_interface.Limb.Quaternion(w=ori[0],x=ori[1],y=ori[2],z=ori[3])
        #Get increment from haptic device
        alpha = 0.001
        delta_pos = alpha*np.matmul(baxter_transform,phantom.hd_vel)
        #print(delta_pos)
        #log_haptic.append([delta_pos[0],delta_pos[1],delta_pos[2]])
        if not phantom.hd_button1:
            #Baxter's new position
            new_pos = Point(cur_pos.x+delta_pos[0],cur_pos.y+delta_pos[1],cur_pos.z+delta_pos[2])
            des_pos = new_pos
            #IK pose
            #pose= PoseStamped(header=hdr,pose=Pose(position= new_pos,orientation=cur_ori))
            pose= PoseStamped(header=hdr,pose=Pose(position= new_pos,orientation=des_ori))
            #seed_angle = JointState(header=hdr,name=limb.joint_names(),position=np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()]))
            ikreq.pose_stamp.append(pose)
            #ikreq.seed_angles.append(seed_angle)
            resp = iksvc(ikreq)

            if (resp.isValid[0]):
                print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                #print limb_joints
            else:
                print("INVALID POSE - No Valid Joint Solution Found.")
                new_pos = cur_pos
                limb_joints = old_joint_angles

            # Baxter joint control
            #for j in limb.joint_names():
            #    limb_joints[j] = limb_joints[j]*0.5 + old_joint_angles[j]*0.5

            #Log data
            #log_joint_angles.append([limb_joints[j] for j in limb.joint_names()])
            #log_pos.append([new_pos.x,new_pos.y,new_pos.z])

            #Move Baxter Joint
            #des_joint_angles = np.asarray([old_joint_angles[j] for j in limb.joint_names()])+ [np.sin(x)*.01,0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x)]

            x+=0.01
            #des_joint_angles = old_joint_angles+np.asarray([0,4*sin(2*3.14*x),0,0,0,0,0])
            #limb_joints = dict(zip(limb.joint_names(),des_joint_angles))
            limb.set_joint_positions(limb_joints,raw=True)
            ikreq.pose_stamp.pop()
            #ikreq.seed_angles.pop()
        else:
            #Reset R_offset
            hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off = R_offset(hd_transform_0,b_transform_0)
        rate.sleep()

    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
