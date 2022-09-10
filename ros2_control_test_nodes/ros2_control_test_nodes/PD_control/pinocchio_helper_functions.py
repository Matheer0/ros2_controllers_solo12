import pinocchio as pin
import numpy as np
import os
from pathlib import Path


class PinocchioHelperFunctions:
    """
    Pinocchio functions used by the PD controller
    """

    def __init__(self):
        # load the URDF model
        urdf_path = os.path.join(Path(__file__).resolve().parents[4].absolute(), "ros2_control_solo", "ros2_description_solo", "urdf",
                                 "solo12.urdf")
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

    def forward_kinematics(self, q):
        foot_positions = np.zeros((4, 3))  # 4x3
        q_list = np.array(list(q))
        pin.forwardKinematics(self.model, self.data, q_list)
        curr_foot = 0
        print(self.model.names)
        print(self.data.oMi)
        for i, (name, oMi) in enumerate(zip(self.model.names, self.data.oMi)):
            print(("{:<24} : {: .2f} {: .2f} {: .2f}"
                   .format(name, *oMi.translation.T.flat)))
            if i % 5 == 0:
                foot_positions[curr_foot] = oMi.translation.T.flat
                curr_foot += 1
        return foot_positions

    def get_mass_matrix(self, q, v):
        M = pin.crba(self.model, self.data, q)
        return M

    def get_h(self, q, v):
        aq0 = np.zeros(self.model.nv)
        v = np.zeros(self.model.nv)
        return pin.rnea(self.model, self.data, q, v, aq0)

    def get_jacobian(self):
        return pin.getJointJacobian(self.model, self.data, reference_frame=pin.ReferenceFrame.WORLD)

    def get_jacobian_dot(self, q, v, id_ankle):
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        return pin.getJointJacobianTimeVariation(self.model, self.data, id_ankle,
                                                 reference_frame=pin.ReferenceFrame.WORLD)
