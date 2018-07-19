import time
import struct
import sys
import copy

import rospy
import rospkg
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
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)

from baxter_interface import CHECK_VERSION
from time import sleep

baxter_transform = np.asarray([
                        [0,0,1],
                        [1,0,0],
                        [0,1,0],
                        ])

delta_eps = 0.1

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

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=-.504, z=0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=-0.25, z=0.91)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    #Load Maze Table SDF
    maze_table_xml = ''
    with open ("/home/arclab/model_editor_models/L_table_smooth/model.sdf", "r") as maze_table_file:
        maze_table_xml = maze_table_file.read().replace('\n','')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("L_table_smooth", maze_table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        #resp_delete = delete_model("Table_maze2")
        #resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    log_joint_angles = []
    log_pos = []
    log_haptic = []
    rospy.init_node('Control_baxter')

    phantom = haptic_pos()

    #Creating the Publisher for rviz visualization
    rviz_pub = rospy.Publisher('hd_point',PoseStamped,queue_size=10)
    hdr_p = Header(stamp=rospy.Time.now(), frame_id='/world')

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Limb initialization
    limb = baxter_interface.Limb('right')
    limb.move_to_neutral()

    neutral_ori = limb.endpoint_pose()['orientation']

    #Spawn tables
    load_gazebo_models()

    #Gripper initialization
    gripper_r = baxter_interface.Gripper('right')

    #IK Service
    ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    #Communication rate - 1kHz
    rate = rospy.Rate(100)
    def reset_baxter():
        np.save('log_joint_angles.npy',np.asarray(log_joint_angles))
        #np.save('log_pos.npy',np.asarray(log_pos))
        #np.save('log_haptic.npy',np.asarray(log_haptic))
        limb.move_to_neutral()
        rs.disable()
        delete_gazebo_models()
    x = 0
    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])

    R_off = R_offset(hd_transform_0,b_transform_0)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x

    rospy.on_shutdown(reset_baxter)

    while not rospy.is_shutdown():
        cur_joint_angles = limb.joint_angles()
        cur_pos = limb.endpoint_pose()['position']
        hd_ori = np.matmul(np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3]),R_off)

        ori = mat2quat(hd_ori)
        cur_ori = limb.endpoint_pose()['orientation']
        #Get increment from haptic device
        alpha = 0.001
        delta_pos = alpha*np.matmul(baxter_transform,phantom.hd_vel)
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
        #print(delta_pos)
        #log_haptic.append([delta_pos[0],delta_pos[1],delta_pos[2]])

        if phantom.hd_button2:
            gripper_r.open()
        else:
            gripper_r.close()

        if not phantom.hd_button1:
            joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
            des_x = hd_x + x_off
            #des_ori  = baxter_interface.Limb.Quaternion(w=ori[0],x=ori[1],y=ori[2],z=ori[3])

            #Fixed orientation
            des_ori  = neutral_ori

            p_pos = Point(des_x[0],des_x[1],des_x[2])
            p_pose= PoseStamped(header=hdr_p,pose=Pose(position= p_pos,orientation= des_ori))
            rviz_pub.publish(p_pose)
            #Baxter's new position
            #new_pos = Point(cur_pos.x+delta_pos[0],cur_pos.y+delta_pos[1],cur_pos.z+delta_pos[2])
            des_pos = Point(des_x[0],des_x[1],des_x[2])

            #IK pose
            #pose= PoseStamped(header=hdr,pose=Pose(position= new_pos,orientation=cur_ori))
            pose= PoseStamped(header=hdr,pose=Pose(position= des_pos,orientation=des_ori))
            #seed_angle = JointState(header=hdr,name=limb.joint_names(),position=np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()]))
            ikreq.pose_stamp.append(pose)
            #ikreq.seed_angles.append(seed_angle)
            resp = iksvc(ikreq)
            #limb_joints = dict(zip(limb.joint_names(),joint_angles))
            #limb.set_joint_positions(limb_joints,raw=True)
            #sleep(0.075)
            if (resp.isValid[0]):
                print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                des_joint_angle = np.asarray(resp.joints[0].position)
                if np.sum(np.abs(des_joint_angle-joint_angles))>delta_eps:
                    joint_angles = des_joint_angle*0.5+joint_angles*0.5
                    #limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                    limb_joints = dict(zip(resp.joints[0].name, joint_angles))
                    limb.set_joint_positions(limb_joints,raw=True)
                    #sleep(0.1)
                    #print limb_joints
            else:
                print("INVALID POSE - No Valid Joint Solution Found.")

            #Log data
            #log_pos.append([new_pos.x,new_pos.y,new_pos.z])

            #Move Baxter Joint
            #des_joint_angles =  np.asarray([np.sin(x)*.01,0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x)])+np.pi/4
            #des_joint_angles = np.asarray([cur_joint_angles[j] for j in limb.joint_names()])+ [np.sin(x)*.01,0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x)]
            #log_joint_angles.append(des_joint_angles)
            #x+=0.001
            #des_joint_angles = cur_joint_angles+np.asarray([0,4*sin(2*3.14*x),0,0,0,0,0])
            #limb_joints = dict(zip(limb.joint_names(),des_joint_angles))

            #limb.set_joint_positions(limb_joints,raw=True)
            ikreq.pose_stamp.pop()

        else:
            while phantom.hd_button1:
                pass

            #Reset R_offset
            hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off = R_offset(hd_transform_0,b_transform_0)

            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            x_off = b_x - hd_x


        rate.sleep()

    limb.move_to_neutral()
    rs.disable()

if __name__ == "__main__":
    main()
