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
from visualization_msgs.msg import Marker

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

def caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T):
    #Gains of PD controller
    K = np.diag([30,30,30,1,1,1])*0.5
    K_v = np.diag([1,1,1,1,1,1])
    #Gains of null controller
    K_null = np.diag([0,0,1,0,0,0,0])*10
    K_v_null = np.diag([0,0,1,0,0,0,0])

    #Baxter's orientation
    b_q = limb.endpoint_pose()['orientation']
    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    b_tranform = quat2mat([b_q.w,b_q.x,b_q.y,b_q.z])

    delta_R = np.matmul(b_tranform.T,des_R)
    delta_theta = np.asarray(transfE.mat2euler(delta_R)).reshape((3,1))
    delta_x = np.asarray(des_x-b_x).reshape((3,1))
    delta_pos = np.concatenate((delta_x,delta_theta))

    des_vel = np.concatenate((hd_l_vel,hd_a_vel))
    b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
    b_end_vel = np.matmul(J_T.T,b_joint_vel)
    delta_vel = des_vel.reshape((6,1)) - b_end_vel.reshape((6,1))
    des_force = np.matmul(K,delta_pos)+np.matmul(K_v,delta_vel)

    des_joint_torques = np.matmul(J_T,des_force)

    #Null space torque
    des_joint_angles = np.asarray([0,0,0,0,0,0,0])
    b_joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
    des_joint_torque_null  = np.matmul(K_null,des_joint_angles-b_joint_angles) + np.matmul(K_v_null,-b_joint_vel)

    #Finding the projection operator for the null torque opreration
    J_T_pinv = np.matmul(np.linalg.inv(np.matmul(J_T.T,J_T)),J_T.T)
    P_null = np.eye(7) - np.matmul(J_T,J_T_pinv)
    T_null = np.asarray(np.matmul(P_null,des_joint_torque_null)).reshape((7,))
    return des_joint_torques+T_null


def joint_limit_test(b_joint_angles):
    #A function to test the joint limit of the system
    eps = 0.01
    joint_max = np.asarray([1.7016,1.047,3.0541,2.618,3.059,2.094,3.059]) - eps
    joint_min = np.asarray([-1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059]) +eps
    if(any(b_joint_angles<joint_min) or any(b_joint_angles>joint_max)):
        print('Joint limit reached')

def main():
    log_x_des = []
    log_x_b = []
    log_hd = []
    log_joint_torque = []
    log_delta_theta = []
    rospy.init_node('Control_baxter')

    #Creating the Publisher for rviz visualization
    rviz_pub = rospy.Publisher('hd_point',PoseStamped,queue_size=10)
    hdr = Header(stamp=rospy.Time.now(), frame_id='/world')

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

    rospy.on_shutdown(reset_baxter)

    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
    R_off = R_offset(hd_transform_0,b_transform_0)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x
    while not rospy.is_shutdown():
        joint_angles = limb.joint_angles()

        #Get posd_t from haptic device
        hd_ori = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))

        if not phantom.hd_button1:
            #Baxter's deisred position
            des_R = np.matmul(hd_ori,R_off)
            des_x = hd_x + x_off
        else:
            print("Button 1 Pressed")
            while phantom.hd_button1:
                J_T = kin.jacobian(pos=[0,0,0]).T
                hd_l_vel = np.zeros((3,1))
                hd_a_vel = np.zeros((3,1))
                des_joint_torques = caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T)

                tor_lim = np.asarray([50,50,50,50,15,15,15])*2
                b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
                des_joint_torques = np.diag(np.clip(des_joint_torques,-tor_lim,tor_lim)).reshape((7,))
                des_joint_torques = des_joint_torques+grv_comp.gravity_torque/100
                limb_torques = dict(zip(limb.joint_names(),des_joint_torques))
                limb.set_joint_torques(limb_torques)

            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            x_off = b_x - hd_x
            hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off = R_offset(hd_transform_0,b_transform)

        des_ori = mat2quat(des_R)
        pos = Point(des_x[0],des_x[1],des_x[2])
        ori = Quaternion(w = des_ori[0], x = des_ori[1], y = des_ori[2], z = des_ori[3])
        pose= PoseStamped(header=hdr,pose=Pose(position= pos,orientation= ori))
        rviz_pub.publish(pose)

        J_T = kin.jacobian(pos=[0,0,0]).T

        #Getting the haptic velocity
        hd_l_vel = np.matmul(baxter_transform,phantom.hd_vel)*0.001
        hd_a_vel = np.matmul(baxter_transform,phantom.hd_ang_vel)

        des_joint_torques = caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T)

        #clip the toruqes:
        tor_lim = np.asarray([50,50,0,50,15,15,15])*2
        des_joint_torques = np.diag(np.clip(des_joint_torques,-tor_lim,tor_lim)).reshape((7,))

        des_joint_torques = des_joint_torques+grv_comp.gravity_torque/100
        #print("Total Torque: {}".format(des_joint_torques))

        log_joint_torque.append(des_joint_torques)
        limb_torques = dict(zip(limb.joint_names(),des_joint_torques))
        limb.set_joint_torques(limb_torques)
        b_joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
        print("Joint angles {}".format(b_joint_angles))
        joint_limit_test(b_joint_angles)
        #print(limb_torques)
        #rospy.spin()
        rate.sleep()
    print()
    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
