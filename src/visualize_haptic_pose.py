import time
import rospy
import baxter_interface
from math import sin
import numpy as np

import transforms3d.euler as transfE

from transforms3d.quaternions import (
    mat2quat,
    quat2mat
    )

from baxter_pykdl import baxter_kinematics

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

from visualizations_msgs.msg import Marker

from baxter_core_msgs.msg import SEAJointState

from baxter_interface import CHECK_VERSION

def R_offset(hd_transform_0,b_transform_0):
    return np.matmul(hd_transform_0.T,b_transform_0)

baxter_transform = np.asarray([
                        [0,0,1],
                        [1,0,0],
                        [0,1,0]
                        ])

class haptic_pos:
    def __init__(self):
        self.hd_vel = np.zeros((3,1))
        self.hd_ang_vel = np.zeros((3,1))
        self.hd_transform = np.zeros((4,4))
        self.hd_position = np.zeros((3,1))
        self.hd_button1 = 0
        self.hd_button2 = 0

    def callback(self,data_stream):
        self.hd_transform = np.reshape(data_stream.data[0:16],(4,4),order='F')
        self.hd_vel = np.asarray(data_stream.data[16:19])
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])

        #self.hd_position = np.asarray(data_stream.data[22:25])
        if data_stream.data[22]==1:
            self.hd_button1 = True
        else:
            self.hd_button1 = False
        if data_stream.data[23]==1:
            self.hd_button2 = True
        else:
            self.hd_button2 = False

def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)

def main():
    rospy.init_node('HapticPoint')

    #Creating the Publisher for rviz visualization
    rviz_pub = rospy.Publisher('hd_point',Marker,queue_size=10)


    #Creating the Point
    hd_point = Marker()
    hd_point.header.frame_id = '/world'
    hd_point.type = Marker.POINTS
    hd_point.scale.x = 0.45
    hd_point.scale.y = 0.45
    hd_point.color.r = 0.0
    hd_point.color.g = 0.5
    hd_point.color.b = 0.5
    hd_point.color.a = 1.0


    #Initalize Subscriber for the Haptic device
    phantom = haptic_pos()
    rospy.Subscriber('pose_msg',Float32MultiArray,phantom.callback,queue_size= 10)
    grv_comp = gravity_compensation()

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Limb initialization
    limb = baxter_interface.Limb('right')
    limb.set_command_timeout(1)
    limb.move_to_neutral()

    #Initializing pyKDL for Jacobian
    kin = baxter_kinematics('right')

    #Communication rate - 1kHz
    rate = rospy.Rate(1000)

    def reset_baxter():
        limb.move_to_neutral()
        rs.disable()

    K = np.diag([30,30,30,1,1,1])*0.5
    K_v = np.diag([1,1,1,1,1,1])

    alpha = 1
    rospy.on_shutdown(reset_baxter)

    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
    R_off = R_offset(hd_transform_0,b_transform_0)
    R_off = mat2quat(R_off)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x*alpha
    while not rospy.is_shutdown():
        joint_angles = limb.joint_angles()

        #Get posd_t from haptic device
        hd_ori = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
        des_ori = np.matmul(hd_ori,R_off)
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))

        #Baxter's orientation
        b_q = limb.endpoint_pose()['orientation']
        b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
        b_tranform = quat2mat([b_q.w,b_q.x,b_q.y,b_q.z])

        #print(np.isclose(np.matmul(J_T.T,b_joint_vel),b_endpoint_vel))
        #Baxter's deisred position
        if not phantom.hd_button1:
            des_R = np.matmul(hd_ori,R_off)
            des_x = alpha*hd_x + x_off
        else:
            print("Button 1 Pressed")
            while phantom.hd_button1:
                J_T = kin.jacobian(pos=[0,0,0]).T
                b_q = limb.endpoint_pose()['orientation']
                b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
                b_tranform = quat2mat([b_q.w,b_q.x,b_q.y,b_q.z])
                delta_R = np.matmul(b_tranform.T,hd_ori)
                delta_theta = np.asarray(transfE.mat2euler(delta_R)).reshape((3,1))
                delta_x = np.asarray(des_x-b_x).reshape((3,1))
                delta_pos = np.concatenate((delta_x,delta_theta))

                b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
                b_end_vel = np.matmul(J_T.T,b_joint_vel)
                delta_vel =  - b_end_vel.reshape((6,1))

                des_force = np.matmul(K,delta_pos)+np.matmul(K_v,delta_vel)*10
                des_joint_torques = np.matmul(J_T,des_force)

                tor_lim = np.asarray([50,50,50,50,15,15,15])*2
                b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
                des_joint_torques = np.diag(np.clip(des_joint_torques,-tor_lim,tor_lim)).reshape((7,))
                des_joint_torques = des_joint_torques+grv_comp.gravity_torque/100
                limb_torques = dict(zip(limb.joint_names(),des_joint_torques))
                limb.set_joint_torques(limb_torques)

            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            hd_transform_0 = phantom.hd_transform[0:3,0:3]
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            x_off = b_x - hd_x*alpha
            R_off = R_offset(hd_transform_0,b_transform_0)

        des_ori = mat2quat(des_R)
        hd_point.pose.orientation.w = des_ori[0]
        hd_point.pose.orientation.x = des_ori[1]
        hd_point.pose.orientation.y = des_ori[2]
        hd_point.pose.orientation.z = des_ori[3]

        hd_point.pose.position.x = des_x[0]
        hd_point.pose.position.y = des_x[1]
        hd_point.pose.position.z = des_x[2]

        J_T = kin.jacobian(pos=[0,0,0]).T
        delta_R = np.matmul(b_tranform.T,des_ori)
        delta_theta = np.asarray(transfE.mat2euler(delta_R)).reshape((3,1))
        delta_x = np.asarray(des_x-b_x).reshape((3,1))
        delta_pos = np.concatenate((delta_x,delta_theta))
        #delta_pos = np.concatenate((alpha*hd_vel,np.zeros((3,1))))
        rviz_pub.publish(hd_point)

        rate.sleep()
    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
