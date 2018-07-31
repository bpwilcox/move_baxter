
class log_data_position(object):
    def __init__(self):
        self.haptic_pos = []
        self.baxter_pos = []
        self.baxter_joint_angles = []

    def add_data(self,haptic_pos,baxter_pos,joint_angles):
        self.haptic_pos.append(haptic_pos)
        self.baxter_pos.append(baxter_pos)
        self.baxter_joint_angles.append(joint_angles)


class log_data_forces(log_data_position):
    def __init__(self):
        log_data_position.__init__(self)
        self.torque = []
        self.delta_pos = []
        self.delta_vel = []
        self.force = []

    def add_data_force(self,torque,force,delta_pos,delta_vel):
        self.torque.append(torque)
        self.force.append(force)
        self.delta_pos.append(delta_pos)
        self.delta_vel.append(delta_vel)
