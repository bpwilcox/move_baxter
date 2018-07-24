'''
This module consist of all the utility function that is required to implement
position control for Baxter
'''
import numpy as np

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
        for i in range(5):
            self.joint_angles.append(joint_angle)

    def add_joint_angle(self, joint_angle):
        self.joint_angles.pop()
        self.joint_angles.insert(0, joint_angle)

    def get_joint_angle(self):
        FIR_lpf = np.asarray([0.2, 0.2, 0.2, 0.2, 0.2])
        return np.matmul(FIR_lpf, np.asarray(self.joint_angles))

def scale_x(x):
    '''
    A function to scale from the haptic space to the baxter space. Note that the
    haptic space in non-uniform, check documentation for more information.
    '''
    scale_mat = np.diag([1.0/160, 1.0/70, 1.0/200])
    # scale_mat = np.eye(3)
    return np.matmul(scale_mat, x)
