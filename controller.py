from adrc_joint_controller import ADRCJointController
import numpy as np

class ADRC_controller():
    def __init__(self, params, Tp, origin):
        self.joint_controllers = []
        for param in params:
            self.joint_controllers.append(ADRCJointController(*param, Tp, origin))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        u = []
        for i, controller in enumerate(self.joint_controllers):
            u.append(controller.calculate_control([x[i], x[i+2]], q_d[i], q_d_dot[i], q_d_ddot[i], i))
        u = np.array(u)[:, np.newaxis]
        return u