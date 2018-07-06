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


class gravity_compensation:
    def __init__(self):
        self.gravity_torque = np.zeros((7))
        node_right = 'robot/limb/right/gravity_compensation_torques'
        rospy.Subscriber(node_right,SEAJointState,self.callback)
    def callback(self,data_stream):
        self.gravity_torque = np.asarray(data_stream.gravity_model_effort[0:7])

def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)

def main():
    log_x_des = []
    log_x_b = []
    log_hd = []
    log_joint_torque = []
    log_delta_theta = []
    rospy.init_node('Control_baxter')
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
        np.save('log_joint_torque.npy',np.asarray(log_joint_torque))
        np.save('log_x_des.npy',np.asarray(log_x_des))
        np.save('log_x_b.npy',np.asarray(log_x_b))
        np.save('log_hd.npy',np.asarray(log_hd))
        np.save('log_delta_theta.npy',np.asarray(log_delta_theta))
        limb.move_to_neutral()
        rs.disable()
    x = 0
    #K = 100
    K = np.diag([30,30,30,1,1,1])
    #K_v = np.diag([1,1,1,1,1,1,1])*10
    K_v = np.diag([1,1,1,1,1,1])

    alpha = 1
    rospy.on_shutdown(reset_baxter)

    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])

    R_off = R_offset(hd_transform_0,b_transform_0)
    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x*alpha
    while not rospy.is_shutdown():
        J_T = kin.jacobian(pos=[0,0,0]).T
        M_ee = kin.cart_inertia()
        #print(J_T)
        joint_angles = limb.joint_angles()

        #Get posd_t from haptic device
        hd_ori = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
        des_ori = np.matmul(hd_ori,R_off)
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
        #print(hd_x)
        #Baxter's orientation
        b_q = limb.endpoint_pose()['orientation']
        b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
        b_tranform = quat2mat([b_q.w,b_q.x,b_q.y,b_q.z])

        #print(np.isclose(np.matmul(J_T.T,b_joint_vel),b_endpoint_vel))
        #Baxter's deisred position
        if not phantom.hd_button1:
            des_R = np.matmul(hd_ori,R_off)
            des_x = alpha*hd_x + x_off
            #print(hd_x,des_x)
            log_x_des.append(des_x)
            log_hd.append(hd_x)
            log_x_b.append(b_x)

        else:
            print("Button 1 Pressed")
            while phantom.hd_button1:
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

        delta_R = np.matmul(b_tranform.T,des_ori)
        delta_theta = np.asarray(transfE.mat2euler(delta_R)).reshape((3,1))
        log_delta_theta.append(delta_theta)
        delta_x = np.asarray(des_x-b_x).reshape((3,1))
        delta_pos = np.concatenate((delta_x,delta_theta))
        #delta_pos = np.concatenate((alpha*hd_vel,np.zeros((3,1))))

        #Finding the velocity error
        hd_l_vel = np.matmul(baxter_transform,phantom.hd_vel)*0.001
        hd_a_vel = np.matmul(baxter_transform,phantom.hd_ang_vel)

        des_vel = np.concatenate((hd_l_vel,hd_a_vel))
        b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
        b_end_vel = np.matmul(J_T.T,b_joint_vel)
        delta_vel = des_vel.reshape((6,1)) - b_end_vel.reshape((6,1))

        des_acc = np.matmul(K,delta_pos)+np.matmul(K_v,delta_vel)

        #des_joint_torques = np.matmul(J_T,np.matmul(M_ee,des_acc))
        des_joint_torques = np.matmul(J_T,des_acc)
        print("Delta x: {}\n Delta vel: {}\n".format(delta_pos,delta_vel))

        #Finding the torque for the secondary controller
        b_q = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
        K_null = np.diag([0,0,1,0,0,0,0])
        #This is to keep the E_0 joint at 0deg
        des_joint_torque_null  = np.matmul(K_null,b_q)

        #Finding the projection operator for the null torque opreration
        J_T_pinv = np.matmul(kin.cart_inertia(),np.matmul(kin.jacobian(),np.linalg.inv(kin.inertia())))
        P_null = np.eye(7) - np.matmul(J_T,J_T_pinv)
        T_null = np.asarray(np.matmul(P_null,des_joint_torque_null)).reshape((7,))

        #clip the toruqes:
        #tor_lim = 1000
        tor_lim = np.asarray([50,50,0,50,15,15,15])*2
        des_joint_torques = np.diag(np.clip(des_joint_torques,-tor_lim,tor_lim)).reshape((7,))
        #des_joint_torques = des_joint_torques-np.matmul(K_v,b_joint_vel)+grv_comp.gravity_torque/100
        #print(des_joint_torques)
        des_joint_torques = des_joint_torques+grv_comp.gravity_torque/100 + T_null
        print("Total Torque: {}".format(des_joint_torques))

        #des_joint_torques = np.zeros(7)#+grv_comp.gravity_torque/100
        #print("Rotation Matrix {} \n Orientation {} joint torques {}".format(delta_R,delta_theta.T,des_joint_torques.T))
        #print(grv_comp.gravity_torque)
        #print(des_joint_torques.T)
        log_joint_torque.append(des_joint_torques)
        limb_torques = dict(zip(limb.joint_names(),des_joint_torques))
        limb.set_joint_torques(limb_torques)

        #print(limb_torques)
        #rospy.spin()
        rate.sleep()
    print()
    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()