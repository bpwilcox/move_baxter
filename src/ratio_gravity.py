import rospy
from baxter_core_msgs.msg import SEAJointState
import numpy as np

class gravity_compensation:
    def __init__(self):
        self.est_gravity_effort = np.zeros((7))
        self.act_effort = np.zeros((7))
        node_right = 'robot/limb/right/gravity_compensation_torques'
        rospy.Subscriber(node_right,SEAJointState,self.callback)
    def callback(self,data_stream):
        self.est_gravity_effort = np.asarray(data_stream.gravity_model_effort[0:7])
        self.act_effort = np.asarray(data_stream.actual_effort[0:7])

def main():
    test  = gravity_compensation()
    rospy.init_node('ratio_test')
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        r = test.act_effort/test.est_gravity_effort
        print("Acual effort {}".format(test.act_effort))
        print("Estimated gravity torque {}".format(test.est_gravity_effort))
        print("Ratio {}".format(r))
        rate.sleep()
if __name__=='__main__':
    main()
