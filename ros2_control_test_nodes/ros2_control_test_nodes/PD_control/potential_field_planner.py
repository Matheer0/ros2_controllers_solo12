
class PotentialFieldPlanner:
    """
    Potential field planner used by PD controller
    """

    def __init__(self, k_attr, delta_t, pos_target):
        self.k_attr = k_attr
        self.delta_t = delta_t
        self.pos_target = pos_target

    def compute_ref_vel(self, pos_fbk):
        return self.k_attr * (self.pos_target - pos_fbk)

    def compute_ref_pos(self, pos_fbk):
        return pos_fbk + self.compute_ref_vel(pos_fbk) * self.delta_t
