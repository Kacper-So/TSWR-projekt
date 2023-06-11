import numpy as np

class controller():
    def __init__(self, origin):
        self.origin = origin

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        u = [0, 0, 0]
        return q_d