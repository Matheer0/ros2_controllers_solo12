import numpy as np
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
import os
from pathlib import Path


class Solo12Config:
    pin_robot = None

    def __init__(self):

        self.base_link_name = "base_link"  # newly added for state estimation

        urdf_path = os.path.join(Path(__file__).resolve().parents[4].absolute(), "ros2_control_solo",  "ros2_description_solo", "urdf",
                                 "solo12_simulation.urdf")
        meshes_path = os.path.join(Path(__file__).resolve().parents[4].absolute(), "ros2_control_solo", "ros2_description_solo")
        print(f'URDF_PATH = {urdf_path}')
        print(f'meshes_path = {meshes_path}')
        Solo12Config.pin_robot = RobotWrapper.BuildFromURDF(urdf_path, meshes_path, se3.JointModelFreeFlyer())
        # Solo12Config.pin_robot = se3.buildModelFromUrdf(self.urdf_path)
        # The inertia of a single blmc_motor.
        motor_inertia = 0.0000045
        # The motor gear ratio.
        motor_gear_ration = 9.0
        Solo12Config.pin_robot.model.rotorInertia[6:] = motor_inertia
        Solo12Config.pin_robot.model.rotorGearRatio[6:] = motor_gear_ration

        self.robot_model = Solo12Config.pin_robot.model
        mass = np.sum([i.mass for i in self.robot_model.inertias])
        base_name = self.robot_model.frames[2].name

        # End effectors informations
        shoulder_ids = []
        end_eff_ids = []
        shoulder_names = []
        end_effector_names = []
        for leg in ["FL", "FR", "HL", "HR"]:
            shoulder_ids.append(self.robot_model.getFrameId(leg + "_HAA"))
            shoulder_names.append(leg + "_HAA")
            end_eff_ids.append(self.robot_model.getFrameId(leg + "_FOOT"))
            end_effector_names.append(leg + "_FOOT")

        self.should_ids = shoulder_ids
        self.end_eff_ids = end_eff_ids
        self.shoulder_names = shoulder_names
        self.end_effector_names = end_effector_names

        nb_ee = len(end_effector_names)
        hl_index = self.robot_model.getFrameId("HL_ANKLE")
        hr_index = self.robot_model.getFrameId("HR_ANKLE")
        fl_index = self.robot_model.getFrameId("FL_ANKLE")
        fr_index = self.robot_model.getFrameId("FR_ANKLE")

        self.nb_ee = nb_ee

        # The number of motors, here they are the same as there are only revolute
        # joints.
        nb_joints = self.robot_model.nv - 6
        joint_names = [
            "FL_HAA",
            "FL_HFE",
            "FL_KFE",
            "FR_HAA",
            "FR_HFE",
            "FR_KFE",
            "HL_HAA",
            "HL_HFE",
            "HL_KFE",
            "HR_HAA",
            "HR_HFE",
            "HR_KFE",
        ]

        # Mapping between the ctrl vector in the device and the urdf indexes.
        urdf_to_dgm = tuple(range(12))

        map_joint_name_to_id = {}
        map_joint_limits = {}
        for i, (name, lb, ub) in enumerate(
                zip(
                    self.robot_model.names[1:],
                    self.robot_model.lowerPositionLimit,
                    self.robot_model.upperPositionLimit,
                )
        ):
            map_joint_name_to_id[name] = i
            map_joint_limits[i] = [float(lb), float(ub)]

        # Define the initial state.
        self.initial_configuration = (
                [0.2, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
                + 2 * [0.0, 0.8, -1.6]
                + 2 * [0.0, -0.8, 1.6]
        )
        self.initial_velocity = (8 + 4 + 6) * [
            0,
        ]

        q0 = zero(self.robot_model.nq)
        q0[:] = self.initial_configuration
        v0 = zero(self.robot_model.nv)
        a0 = zero(self.robot_model.nv)

        base_p_com = [0.0, 0.0, -0.02]

        self.rot_base_to_imu = np.identity(3)
        self.r_base_to_imu = np.array([0.10407, -0.00635, 0.01540])

        Solo12Config.pin_robot = self.pin_robot

        self.mass = np.sum([i.mass for i in self.robot_model.inertias])

        # TODO: refactor this
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.ctrl_path = os.path.join(dir_path, "..", "..", "config", "impedance_ctrl.yaml")

    @classmethod
    def buildRobotWrapper(cls):
        return Solo12Config.pin_robot

    def get_feet_pos(self, q):
        Solo12Config.pin_robot.framesForwardKinematics(q)
        return np.array([self.pin_robot.data.oMf[i].translation for i in self.end_eff_ids])
