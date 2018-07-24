import rospy
import baxter_interface
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

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from baxter_interface import CHECK_VERSION
# from time import sleep

#Custom Modules
from msg import haptic_pos,block_pos
from utils import R_offset,joint_filter,scale_x
from gzb_interface import render_camera,load_gazebo_models,delete_camera,delete_gazebo_models

baxter_transform = np.asarray([
                        [0, 0, 1],
                        [1, 0, 0],
                        [0, 1, 0],
                        ])

delta_eps = 0.1


def main():
    log_joint_angles = []
    # log_pos = []
    # log_haptic = []
    rospy.init_node('Control_baxter')

    phantom = haptic_pos()

    #Creating the Publisher for rviz visualization
    rviz_pub = rospy.Publisher('hd_point', PoseStamped, queue_size=10)
    hdr_p = Header(stamp=rospy.Time.now(), frame_id='/world')

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Limb initialization
    limb = baxter_interface.Limb('right')
    limb.move_to_neutral()

    # neutral_ori = limb.endpoint_pose()['orientation']

    #Spawn tables
    load_gazebo_models()

    # Intialize the joint angle filter
    joint_angle = [limb.joint_angle(joint) for joint in limb.joint_names()]
    # joint_angle_filter = joint_filter(joint_angle)
    # model_pos = block_pos()
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
        np.save('log_b_joint_angles_position.npy', np.asarray(log_joint_angles))
        #np.save('log_pos.npy',np.asarray(log_pos))
        #np.save('log_haptic.npy',np.asarray(log_haptic))
        limb.move_to_neutral()
        rs.disable()
        delete_camera()
        delete_gazebo_models()

    # x = 0
    hd_transform_0 = np.matmul(
        baxter_transform, phantom.hd_transform[0:3, 0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w, b_ori_q.x, b_ori_q.y, b_ori_q.z])

    R_off = R_offset(hd_transform_0, b_transform_0)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform, phantom.hd_transform[0:3, 3]))
    x_off = b_x - hd_x

    rospy.on_shutdown(reset_baxter)
    render_camera()

    while not rospy.is_shutdown():
        # cur_joint_angles = limb.joint_angles()
        # cur_pos = limb.endpoint_pose()['position']
        hd_ori = np.matmul(
            np.matmul(baxter_transform, phantom.hd_transform[0:3, 0:3]), R_off)

        ori = mat2quat(hd_ori)
        # cur_ori = limb.endpoint_pose()['orientation']
        #Get increment from haptic device
        # alpha = 0.001
        # delta_pos = alpha*np.matmul(baxter_transform, phantom.hd_vel)
        hd_x = scale_x(np.matmul(baxter_transform,
                                 phantom.hd_transform[0:3, 3]))
        #print(delta_pos)
        #log_haptic.append([delta_pos[0],delta_pos[1],delta_pos[2]])

        if phantom.hd_button2:
            gripper_r.open()
        else:
            gripper_r.close()
        #print(model_pos.modelInfo.pose[3].position)

        if not phantom.hd_button1:
            joint_angles = np.asarray(
                [limb.joint_angle(joint) for joint in limb.joint_names()])
            log_joint_angles.append(joint_angles)
            des_x = hd_x + x_off
            des_ori  = Quaternion(w=ori[0],x=ori[1],y=ori[2],z=ori[3])

            #Fixed orientation
            # des_ori = neutral_ori

            p_pos = Point(des_x[0], des_x[1], des_x[2])
            p_pose = PoseStamped(header=hdr_p, pose=Pose(
                position=p_pos, orientation=des_ori))
            rviz_pub.publish(p_pose)
            #Baxter's new position
            des_pos = Point(des_x[0], des_x[1], des_x[2])

            #IK pose
            pose = PoseStamped(header=hdr, pose=Pose(
                position=des_pos, orientation=des_ori))
            #seed_angle = JointState(header=hdr,name=limb.joint_names(),position=np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()]))
            ikreq.pose_stamp.append(pose)
            #ikreq.seed_angles.append(seed_angle)
            resp = iksvc(ikreq)

            if (resp.isValid[0]):
                print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                des_joint_angle = np.asarray(resp.joints[0].position)
                #joint_angle_filter.add_joint_angle(resp.joints[0].position)
                #des_joint_angle = joint_angle_filter.get_joint_angle()
                if np.sum(np.abs(des_joint_angle-joint_angles)) > delta_eps:
                    joint_angles = des_joint_angle#*0.5+joint_angles*0.5
                    #limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                    limb_joints = dict(zip(resp.joints[0].name, joint_angles))
                    limb.set_joint_positions(limb_joints, raw=True)
                    #sleep(0.1)
                    #print limb_joints
            else:
                print("INVALID POSE - No Valid Joint Solution Found.")

            # Log data
            # log_pos.append([new_pos.x,new_pos.y,new_pos.z])

            # Move Baxter Joint
            # des_joint_angles =  np.asarray([np.sin(x)*.01,0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x)])+np.pi/4
            # des_joint_angles = np.asarray([cur_joint_angles[j] for j in limb.joint_names()])+ [np.sin(x)*.01,0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x),0.01*np.sin(x)]
            # log_joint_angles.append(des_joint_angles)
            # x+=0.001
            # des_joint_angles = cur_joint_angles+np.asarray([0,4*sin(2*3.14*x),0,0,0,0,0])
            # limb_joints = dict(zip(limb.joint_names(),des_joint_angles))

            # limb.set_joint_positions(limb_joints,raw=True)
            ikreq.pose_stamp.pop()

        else:
            while phantom.hd_button1:
                pass

            # Reset R_offset
            hd_transform_0 = np.matmul(
                baxter_transform, phantom.hd_transform[0:3, 0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat(
                [b_ori_q.w, b_ori_q.x, b_ori_q.y, b_ori_q.z])
            R_off = R_offset(hd_transform_0, b_transform_0)

            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            hd_x = scale_x(np.matmul(baxter_transform,
                                     phantom.hd_transform[0:3, 3]))
            x_off = b_x - hd_x

        rate.sleep()

    limb.move_to_neutral()
    rs.disable()


if __name__ == "__main__":
    main()
