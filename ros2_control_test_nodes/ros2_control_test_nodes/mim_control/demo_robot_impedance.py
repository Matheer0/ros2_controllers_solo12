from ros2_control_test_nodes.robot_properties_solo.Solo12Robot import Solo12Robot
from ros2_control_test_nodes.robot_properties_solo.Solo12Config import Solo12Config
from .robot_impedance_controller import RobotImpedanceController


class Demo:

    def __init__(self):
        # Create a robot instance in the simulator
        robot_config = Solo12Config()
        robot = Solo12Robot()

        # Impedance controller gains
        # kp = robot.nb_ee * [100.0, 100.0, 100.0]
        # kd = robot.nb_ee * [10.0, 10.0, 10.0]
        # kp = robot.nb_ee * [125.0, 125.0, 125.0] # tuned with the crane
        # kd = robot.nb_ee * [3.0, 3.0, 3.0] # tuned with the crane
        # kp = robot.nb_ee * [45.0, 45.0, 45.0]
        # kd = robot.nb_ee * [1.0, 1.0, 1.0]
        kp = robot.nb_ee * [100.0, 100.0, 100.0]
        kd = robot.nb_ee * [5.0, 5.0, 5.0]

        # Desired leg length.
        x_des = robot.nb_ee * [0.0, 0.0, -0.25]
        xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

        # distributing forces to the active end-effectors
        # f = robot.nb_ee * [0.0, 0.0, (robot_config.mass * 9.8) / 4]
        f = robot.nb_ee * [0.0, 0.0, (robot_config.mass * 9.8) / 4]
        # f = robot.nb_ee * [0.0, 0.0, 0.0]

        config_file = robot_config.ctrl_path
        robot_leg_ctrl = RobotImpedanceController(robot, config_file)

        self.robot = robot
        self.robot_leg_ctrl = robot_leg_ctrl
        self.kp = kp
        self.kd = kd
        self.x_des = x_des
        self.xd_des = xd_des
        self.f = f

    def compute_torques(self, q, dq):
        tau = self.robot_leg_ctrl.return_joint_torques(
            q, dq, self.kp, self.kd, self.x_des, self.xd_des, self.f
        )
        return tau
