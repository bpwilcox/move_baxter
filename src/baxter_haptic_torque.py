import rospy
import baxter_interface
from math import sin
import numpy as np
import pickle

import transforms3d.euler as transfE
from gzb_interface import load_gazebo_models,delete_gazebo_models,load_marker,delete_marker

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
    Float32MultiArray,
    Empty
)


from baxter_core_msgs.msg import SEAJointState

from baxter_interface import CHECK_VERSION

# Custom modules
from utils import R_offset
from msg import haptic_pos
from log_data import log_data_forces

LogData = log_data_forces()
baxter_transform = np.asarray([
                        [0,0,1],
                        [1,0,0],
                        [0,1,0]
                        ])

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

def set_goal():
    y = np.random.uniform(-0.9,-0.7,1)
    x = np.random.uniform(0.1,0.35,1)
    z = 0.9
    return np.squeeze([x,y,z])

def caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T,grav_comp):
    #Gains of PD controller

    #K = np.diag([9]*3+[0.08]*3)

    # Gains for Simulator
    K_1 = 100
    K_2 = 8
    K_p_f = np.diag([K_1]*3)
    K_p_w = np.diag([K_2]*3)
    K_v = np.diag([32]*3+[2*np.sqrt(K_2)]*3)
    #Gains of null controller
    K_null = np.diag([0,0,1,0,0,0,0])*10
    K_v_null = np.diag([0,0,1,0,0,0,0])

    """
    # Gains for real robot without Grippers

    K_1 = 80
    K_2 = 8
    K_p_f = np.diag([K_1]*3)
    K_p_w = np.diag([K_2]*3)
    K_v = np.diag([15]*3+[1]*3)
    #Gains of null controller
    K_null = np.diag([0,0,1,0,0,0,0])*10
    K_v_null = np.diag([0,0,1,0,0,0,0])
    """

    #Baxter's orientation
    b_q = limb.endpoint_pose()['orientation']
    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    b_transform = quat2mat([b_q.w,b_q.x,b_q.y,b_q.z])

    euler_seq = 'sxyz'
    delta_R = np.matmul(b_transform.T,des_R)
    delta_theta = np.sin(np.asarray(transfE.mat2euler(delta_R,euler_seq)).reshape((3,1)))
    delta_x = np.asarray(des_x-b_x).reshape((3,1))
    delta_pos = np.concatenate((delta_x,delta_theta))

    des_vel = np.concatenate((hd_l_vel,hd_a_vel))
    b_joint_vel = np.asarray([limb.joint_velocity(name) for name in limb.joint_names()])
    b_end_vel = np.matmul(J_T.T,b_joint_vel)
    delta_vel = des_vel.reshape((6,1)) - b_end_vel.reshape((6,1))

    des_force_p = np.concatenate((np.matmul(K_p_f,delta_x),np.matmul(b_transform,np.matmul(K_p_w,delta_theta))))
    des_force = des_force_p+np.matmul(K_v,delta_vel)
    #print("Desired force {}".format(des_force))
    des_joint_torques = np.matmul(J_T,des_force)
    # print("Desired torque {}".format(des_joint_torques))

    #Null space torque
    des_joint_angles = np.asarray([0,0,0,0,0,0,0])
    b_joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
    des_joint_torque_null  = np.matmul(K_null,des_joint_angles-b_joint_angles) + np.matmul(K_v_null,-b_joint_vel)

    #Finding the projection operator for the null torque opreration
    J_T_pinv = np.matmul(np.linalg.inv(np.matmul(J_T.T,J_T)),J_T.T)
    P_null = np.eye(7) - np.matmul(J_T,J_T_pinv)
    T_null = np.asarray(np.matmul(P_null,des_joint_torque_null)).reshape((7,))
    des_joint_torques = des_joint_torques + grav_comp.gravity_torque*0.2+T_null

    #clip the torques:
    tor_lim = np.asarray([50,50,50,50,15,15,15])*2
    des_joint_torques = np.diag(np.clip(des_joint_torques,-tor_lim,tor_lim)).reshape((7,))
    LogData.add_data_force(des_joint_torques,des_force,delta_pos,delta_vel)
    haptic_q = transfE.mat2euler(des_R,euler_seq)
    baxter_orient = limb.endpoint_pose()['orientation']
    baxter_q = transfE.quat2euler([baxter_orient.w,baxter_orient.x,baxter_orient.y,baxter_orient.z],euler_seq)
    baxter_x = limb.endpoint_pose()['position']
    haptic_pose = list(des_x)+list(haptic_q)
    baxter_pose = [baxter_x.x,baxter_x.y,baxter_x.z]+list(baxter_q)
    baxter_joint_angles = [ limb.joint_angle(joint) for joint in limb.joint_names()]
    LogData.add_data(haptic_pose,baxter_pose,baxter_joint_angles)
    # des_joint_torques = np.zeros((7,))
    limb_torques = dict(zip(limb.joint_names(),des_joint_torques))
    limb.set_joint_torques(limb_torques)
    # print(limb_torques)

def set_joint_positions(limb,joint_positions):
    joint_positions = dict(zip(limb.joint_names(),joint_positions))
    limb.move_to_joint_positions(joint_positions)

def joint_limit_test(b_joint_angles):
    #A function to test the joint limit of the system
    eps = 0.01
    joint_max = np.asarray([1.7016,1.047,3.0541,2.618,3.059,2.094,3.059]) - eps
    joint_min = np.asarray([-1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059]) +eps
    if(any(b_joint_angles<joint_min) or any(b_joint_angles>joint_max)):
        print('Joint limit reached')

def main():
    rospy.init_node('Control_baxter')

    #Creating the Publisher for rviz visualization
    rviz_pub = rospy.Publisher('hd_point',PoseStamped,queue_size=10)
    hdr = Header(stamp=rospy.Time.now(), frame_id='/world')

    #Initalize Subscriber for the Haptic device
    phantom = haptic_pos()
    rospy.Subscriber('pose_msg',Float32MultiArray,phantom.callback,queue_size= 1)

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Limb initialization
    limb = baxter_interface.Limb('right')

    #For safety reasons set timeout for 20 commands
    limb.set_command_timeout(1)
    limb.move_to_neutral()

    #Initializing pyKDL for Jacobian
    kin = baxter_kinematics('right')

    #Communication rate - 1kHz
    rate = rospy.Rate(500)

    # Initialize publisher for zero torques
    pub_gravity = rospy.Publisher('/robot/limb/right/suppress_gravity_compensation',Empty,queue_size=1)
    grv_comp = gravity_compensation()
    

    def reset_baxter():
        limb.exit_control_mode()

        with open('torque_control.pkl', 'wb') as output:
            pickle.dump(LogData,output,pickle.HIGHEST_PROTOCOL)
        delete_marker()
        delete_gazebo_models()
        limb.move_to_neutral()
        rs.disable()

    rospy.on_shutdown(reset_baxter)
    load_gazebo_models()

    # Get the goal and add the marker
    goal =set_goal()
    load_marker(goal)
    hd_transform_0 = phantom.hd_transform[0:3,0:3]
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
    R_off = np.matmul(hd_transform_0.T, b_transform_0)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x

    # pub_gravity.publish()
    while not rospy.is_shutdown():
        joint_angles = limb.joint_angles()

        #Get posd_t from haptic device
        hd_ori = phantom.hd_transform[0:3,0:3]
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))

        if not phantom.hd_button1:
            #Baxter's deisred position
            des_R = np.matmul(hd_ori,R_off)
            des_x = hd_x + x_off
        else:
            print("Button 1 Pressed")
            while phantom.hd_button1:
                J_T = kin.jacobian().T
                hd_l_vel = np.zeros((3,1))
                hd_a_vel = np.zeros((3,1))
                caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T,grv_comp)
                pub_gravity.publish()
            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            print(b_x)
            x_off = b_x - hd_x
            hd_transform = phantom.hd_transform[0:3,0:3]
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off =np.matmul(hd_transform.T,b_transform)

        const = 1/np.sqrt(2)
        R1 = np.asarray([[1,0,0],
                         [0,const,const],
                         [0,-const,const]])
        R2 = np.asarray([[const,0,-const],
                         [0,1,0],
                         [const,0,const]])
        R3 = np.asarray([[const,const,0],
                         [-const,const,0],
                         [0,0,1]])
        # des_R = np.matmul(R3, b_transform_0)
        # des_x = np.asarray([0.8,-0.7,0.1])
        des_ori = mat2quat(des_R)
        pos = Point(des_x[0],des_x[1],des_x[2])
        ori = Quaternion(w = des_ori[0], x = des_ori[1], y = des_ori[2], z = des_ori[3])
        pose= PoseStamped(header=hdr,pose=Pose(position= pos,orientation= ori))
        rviz_pub.publish(pose)

        J_T = kin.jacobian().T

        #Getting the haptic velocity
        hd_l_vel = np.matmul(baxter_transform,phantom.hd_vel)*0.001
        hd_a_vel = np.matmul(baxter_transform,phantom.hd_ang_vel)

        hd_l_vel = np.asarray([0,0,0])
        hd_a_vel = np.asarray([0,0,0])

        caltorque(des_x,des_R,hd_l_vel,hd_a_vel,limb,x_off,R_off,J_T,grv_comp)

        b_joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
        #print("Joint angles {}".format(b_joint_angles))
        joint_limit_test(b_joint_angles)
        pub_gravity.publish()
        rate.sleep()
    print()
    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
