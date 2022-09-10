import numpy as np


class PDController:

    def __init__(self, k, d):
        self.k = k
        self.d = d

    def compute_torques(self, m, h, q_dot_ref, q_dot_fbk, q_ref, q_fbk):
        q_ddot_target = self.d * (q_dot_ref - q_dot_fbk) + self.k * (q_ref - q_fbk)
        tau = np.matmul(m, q_ddot_target) + h
        return tau
