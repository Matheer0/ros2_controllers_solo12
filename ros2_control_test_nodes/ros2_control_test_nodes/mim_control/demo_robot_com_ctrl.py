from ros2_control_test_nodes.robot_properties_solo.Solo12Robot import Solo12Robot
from ros2_control_test_nodes.robot_properties_solo.Solo12Config import Solo12Config
from .robot_impedance_controller import RobotImpedanceController
from .robot_centroidal_controller import RobotCentroidalController


class Demo():

    def __init__(self):
        # Create a robot instance in the simulator.
        robot_config = Solo12Config()
        robot = Solo12Robot()
        mu = 0.9
        kc = [40, 40, 40]
        dc = [20, 20, 20]
        kb = [4, 4, 4]
        db = [4, 4, 4]
        qp_penalty_lin = 3 * [1e6]
        qp_penalty_ang = 3 * [1e6]

        # Desired center of mass position and velocity.
        x_com = [0.0, 0.0, 0.25]
        xd_com = [0.0, 0.0, 0.0]
        # The base should be flat.
        x_ori = [0.0, 0.0, 0.0, 1.0]
        x_angvel = [0.0, 0.0, 0.0]
        # All end-effectors are in contact.
        cnt_array = robot.nb_ee * [1]

        # Impedance controller gains
        kp = robot.nb_ee * [0.0, 0.0, 0.0]  # Disable for now
        kd = robot.nb_ee * [0.0, 0.0, 0.0]
        x_des = robot.nb_ee * [0.0, 0.0, -0.25]  # Desired leg length
        xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

        config_file = robot_config.ctrl_path
        robot_cent_ctrl = RobotCentroidalController(
            robot_config,
            mu=mu,
            kc=kc,
            dc=dc,
            kb=kb,
            db=db,
            qp_penalty_lin=qp_penalty_lin,
            qp_penalty_ang=qp_penalty_ang,
        )
        robot_leg_ctrl = RobotImpedanceController(robot, config_file)

        self.kp = kp
        self.kd = kd
        self.x_com = x_com
        self.xd_com = xd_com
        self.x_ori = x_ori
        self.x_angvel = x_angvel
        self.x_des = x_des
        self.xd_des = xd_des
        self.cnt_array = cnt_array
        self.robot_cent_ctrl = robot_cent_ctrl
        self.robot_leg_ctrl = robot_leg_ctrl

    def compute_torques(self, q, dq):
        # computing forces to be applied in the centroidal space
        w_com = self.robot_cent_ctrl.compute_com_wrench(
            q, dq, self.x_com, self.xd_com, self.x_ori, self.x_angvel
        )
        # distributing forces to the active end effectors
        F = self.robot_cent_ctrl.compute_force_qp(q, dq, self.cnt_array, w_com)
        # passing forces to the mim_control controller
        tau = self.robot_leg_ctrl.return_joint_torques(
            q, dq, self.kp, self.kd, self.x_des, self.xd_des, F
        )
        return tau
