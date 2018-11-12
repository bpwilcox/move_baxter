
'''
This module contains all the class definition for communication with the Haptic
device and ROS
'''
import numpy as np
import rospy

from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates

class haptic_pos:
    '''
    The class for receiving the Haptic device pose
    '''
    def __init__(self):
        self.hd_vel = np.zeros((3, 1))
        self.hd_ang_vel = np.zeros((3, 1))
        self.hd_transform = np.eye(4)
        self.hd_position = np.zeros((3, 1))
        self.hd_button1 = False
        self.hd_button2 = False
        rospy.Subscriber('pose_msg', Float32MultiArray, self.callback)

    def callback(self, data_stream):
        self.hd_transform = np.reshape(
            data_stream.data[0:16], (4, 4), order='F')
        self.hd_vel = np.asarray(data_stream.data[16:19])
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])
        # self.hd_position = np.asarray(data_stream.data[22:25])
        if data_stream.data[22] == 1:
            self.hd_button1 = True
        else:
            self.hd_button1 = False
        if data_stream.data[23] == 1:
            self.hd_button2 = True
        else:
            self.hd_button2 = False

class block_pos:
    '''
    The class definition for getting the pose of models in Gazebo
    '''
    def __init__(self):
        self.modelInfo = []
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    def callback(self, data):
        self.modelInfo = data
