'''
This module consist of all the utility function that is required to implement
position control for Baxter
'''
import numpy as np
from scipy import signal

def R_offset(hd_transform_0, b_transform_0):
    '''
    A function calculate Roffset for the transform matrix
    '''
    return np.matmul(hd_transform_0.T, b_transform_0)

class joint_filter:
    '''
    A low pass filter for the joint control
    '''
    def __init__(self, joint_angle):
        self.joint_angles = []
        b,a = signal.butter(7,0.05,btype='low')
        self.b = b
        self.a = a
        for i in range(30):
            self.joint_angles.append(joint_angle)

    def add_joint_angle(self, joint_angle):
        self.joint_angles.pop()
        self.joint_angles.insert(0, joint_angle)

    def get_joint_angle(self):
        angles = np.asarray(self.joint_angles)
        filter_angles = []
        for i in range(7):
            angle = signal.filtfilt(self.b, self.a, angles[:,i])
            filter_angles.append(angle[-1])
        return filter_angles
'''
class input_filter:
'''
    # A low pass filter for the haptic input
'''
    def __init__(self):
        self.input_pos = []
        for i in range(5):
            self.input_pos.append(np.zeros(5))

    def add_input(self,input):
        self.input_pos.pop()
        self.input_pos.insert(0,input)

    def get_input_pos(self):
        FIR_lpf = np.asarray([0.2, 0.2, 0.2, 0.2, 0.2])
        return np.matmul(FIR_lpf, np.asarray(self.input_pos))
'''

def scale_x(x):
    '''
    A function to scale from the haptic space to the baxter space. Note that the
    haptic space in non-uniform, check documentation for more information.
    '''
    scale_mat = np.diag([1.0/160, 1.0/70, 1.0/200])
    # scale_mat = np.eye(3)
    return np.matmul(scale_mat, x)

def joint_limit_test(b_joint_angles):
    #A function to test the joint limit of the system
    eps = 0.01
    joint_max = np.asarray([1.7016,1.047,3.0541,2.618,3.059,2.094,3.059]) - eps
    joint_min = np.asarray([-1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059]) +eps
    if(any(b_joint_angles<joint_min) or any(b_joint_angles>joint_max)):
        print('Joint limit reached')
