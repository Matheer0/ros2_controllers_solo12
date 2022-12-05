import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt

import numpy as np
from enum import Enum

# PD Controller imports
from ros2_control_test_nodes.PD_control.controllers import PDController
from ros2_control_test_nodes.PD_control.potential_field_planner import PotentialFieldPlanner
from ros2_control_test_nodes.PD_control.pinocchio_helper_functions import PinocchioHelperFunctions

# mim_control_ imports
from ros2_control_test_nodes.mim_control.demo_robot_impedance import Demo as ImpedanceDemo
from ros2_control_test_nodes.mim_control.demo_robot_com_ctrl import Demo as CentroidalDemo

# reactive planner import
from ros2_control_test_nodes.reactive_planners.demo_reactive_planners_solo12_step_adjustment_walk import \
    Demo as ReactivePlannerDemo

# TF2 for the base footprint frame
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from ros2_control_test_nodes.robot_properties_solo.Solo12Config import Solo12Config

plt.rcParams.update({'font.size': 22})

class RobotControllers(Node):

    def __init__(self):
        super().__init__('robot_test_controllers')
        self.joint_config = np.zeros(12)
        self.joint_velocity = np.zeros(12)
        self.joint_effort = np.zeros(12)
        self.robot_pose = np.zeros(7)
        self.robot_twist = np.zeros(6)
        self.k = 10  # k in potential field planner
        self.delta_t = 0.002  # also for the potential field planner

        # dictionary to re-order the numbers read from /joint_states
        self.joint_states_order = {'FL_HAA': 0, 'FL_HFE': 1, 'FL_KFE': 2, 'FR_HAA': 3, 'FR_HFE': 4, 'FR_KFE': 5,
                                   'HL_HAA': 6, 'HL_HFE': 7, 'HL_KFE': 8, 'HR_HAA': 9, 'HR_HFE': 10, 'HR_KFE': 11}

        # Declare all paramters
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("config_desired", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # Read all parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.config_des = np.array(self.get_parameter("config_desired").value)

        # Create joint state publisher, timer, and joint state subscriber
        self.publisher_ = self.create_publisher(Float64MultiArray, "/effort_controllers/commands", 10)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.update_joint_states,
                                                               10)

        # Subscribe to the model_states parameter
        self.link_states_subscriber = self.create_subscription(LinkStates, "/link_states", self.update_link_states, 10)

        # Robot Impedance Demo
        self.impedance_demo = ImpedanceDemo()

        # Robot Centroidal Demo
        self.centroidal_demo = CentroidalDemo()

        # Reactive Planner
        self.reactive_planner_demo = ReactivePlannerDemo()
        temp_q = np.array(
            [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.96, -1.89, -0.02, 0.95, -1.89, 0.01, -0.96, 1.89, -0.02, -0.95,
             1.88])
        self.control_time = 0
        self.reactive_planner_demo.initialize_quadruped_dcm_reactive_stepper(temp_q)

        # Helper functions for the inverse dynamics controller
        self.pin_helper_funcs = PinocchioHelperFunctions()
        self.potential_field_planner = PotentialFieldPlanner(self.k, self.delta_t, self.config_des)
        self.PD_controller = PDController(3000, 300)  # try k = 500

        # States
        self.States = Enum("States", "STAND IMPEDANCE CENTROIDAL WALK RESET_EFFORT PLOT")
        self.curr_state = self.States.RESET_EFFORT

        # Service to start the PD control
        self.srv_PD = self.create_service(Trigger, "trigger_PD", self.trigger_PD_callback)
        # Service to start the impedance control
        self.srv_impedance = self.create_service(Trigger, "trigger_impedance", self.trigger_impedance_callback)
        # Service to start the centroidal control
        self.srv_centroidal = self.create_service(Trigger, "trigger_centroidal", self.trigger_centroidal_callback)
        # Service to start the reactive planner step adjustment walk
        self.srv_reactive_planner = self.create_service(Trigger, "trigger_walk", self.trigger_reactive_planner_callback)
        # Service to reset effort to 0
        self.srv_reset_effort = self.create_service(Trigger, "reset_effort", self.trigger_reset_effort_callback)
        # Service to make a plot of the footstep positions from the reactive planner controller
        # meant to be called after calling the "trigger_walk" service
        self.srv_plot = self.create_service(Trigger, "plot", self.trigger_plot_callback)

        # setup base footprint frame
        self.br = TransformBroadcaster(self)  # transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_config = Solo12Config()

    def timer_callback(self):

        def compute_tau_from_PD():
            # TODO: Put this PD_control class
            m = self.pin_helper_funcs.get_mass_matrix(self.joint_config, self.joint_velocity)  # mass matrix
            h = self.pin_helper_funcs.get_h(self.joint_config, self.joint_velocity)
            # compute reference joint velocity and reference joint position
            q_dot_ref = self.potential_field_planner.compute_ref_vel(self.joint_config)
            q_ref = self.potential_field_planner.compute_ref_pos(self.joint_config)
            # compute torques and send
            tau = self.PD_controller.compute_torques(m, h, q_dot_ref, self.joint_velocity, q_ref, self.joint_config)
            return tau

        def base_frame_rel_footprint():
            # body pose in the footprint frame
            now = rclpy.time.Time()
            trans_fp_to_bl = self.tf_buffer.lookup_transform(
                "footprint",
                "base_link",
                now)
            pose_fp = np.array([trans_fp_to_bl.transform.translation.x,
                                trans_fp_to_bl.transform.translation.y,
                                trans_fp_to_bl.transform.translation.z,
                                trans_fp_to_bl.transform.rotation.x,
                                trans_fp_to_bl.transform.rotation.y,
                                trans_fp_to_bl.transform.rotation.z,
                                trans_fp_to_bl.transform.rotation.w])
            pose_fp[3] = self.robot_pose[3]
            pose_fp[4] = self.robot_pose[4]
            pose_fp[5] = self.robot_pose[5]
            pose_fp[6] = self.robot_pose[6]

            # converting the twist to the footprint frame
            trans_fp_to_wf = self.tf_buffer.lookup_transform(
                "footprint",
                "world",
                now
            )
            rotation_fp_wf = tf_transformations.quaternion_matrix([trans_fp_to_wf.transform.rotation.x,
                                                                   trans_fp_to_wf.transform.rotation.y,
                                                                   trans_fp_to_wf.transform.rotation.z,
                                                                   trans_fp_to_wf.transform.rotation.w])
            linear_twist_fp = np.matmul(rotation_fp_wf[:3, :3], self.robot_twist[:3])
            angular_twist_fp = np.matmul(rotation_fp_wf[:3, :3], self.robot_twist[3:])
            return pose_fp, np.concatenate((linear_twist_fp, angular_twist_fp))

        if self.curr_state == self.States.STAND:
            tau = compute_tau_from_PD()
            msg = Float64MultiArray()
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

        if self.curr_state == self.States.IMPEDANCE:
            msg = Float64MultiArray()
            joint_config_with_base = np.concatenate((self.robot_pose, self.joint_config))
            joint_vel_with_base = np.concatenate((self.robot_twist, self.joint_velocity))
            tau = self.impedance_demo.compute_torques(joint_config_with_base, joint_vel_with_base)
            tau_from_PD = compute_tau_from_PD()
            tau = tau + tau_from_PD
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

        if self.curr_state == self.States.CENTROIDAL:
            msg = Float64MultiArray()
            pose_fp, twist_fp = base_frame_rel_footprint()
            joint_config_with_base = np.concatenate((pose_fp, self.joint_config))
            joint_vel_with_base = np.concatenate((twist_fp, self.joint_velocity))
            tau = self.centroidal_demo.compute_torques(joint_config_with_base, joint_vel_with_base)
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

        if self.curr_state == self.States.WALK:
            # start = self.get_clock().now().to_msg().nanosec
            msg = Float64MultiArray()
            # pose_fp, twist_fp = base_frame_rel_footprint()
            # joint_config_with_base = np.concatenate((pose_fp, self.joint_config))
            # joint_vel_with_base = np.concatenate((twist_fp, self.joint_velocity))
            joint_config_with_base = np.concatenate((self.robot_pose, self.joint_config))
            joint_vel_with_base = np.concatenate((self.robot_twist, self.joint_velocity))
            tau = self.reactive_planner_demo.compute_torques(joint_config_with_base, joint_vel_with_base,
                                                             self.control_time,
                                                             "forward")  # last argument options: forward, turn,
            # desired_feet_pos = self.reactive_planner_demo.get_desired_next_step_pos(joint_config_with_base)
            # self.get_logger().info(f'desired next step feet pos = {desired_feet_pos}')
            msg.data = tau.tolist()
            self.publisher_.publish(msg)
            # end = self.get_clock().now().to_msg().nanosec
            # print(f'duration = {end-start}')
            # self.get_logger().info(f'total time = {end - start}')
            self.control_time += 0.002

        if self.curr_state == self.States.RESET_EFFORT:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_.publish(msg)

        if self.curr_state == self.States.PLOT:
            x_curr_local, x_des_local, feet_pos_des, com_pos = self.reactive_planner_demo.get_pos_feet()
            # compute error
            err = np.sum(np.absolute(x_curr_local[:, 1][100:200] - x_des_local[:, 1][100:200])) + np.sum(np.absolute(x_curr_local[:, 1][300:400] - x_des_local[:, 1][300:400]))
            self.get_logger().info(f'err = {err}')
            time_step = np.arange(0, x_curr_local.shape[0] * 0.001, 0.001)
            # fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(5, 2.7))
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(5, 2.7))
            ax1.plot(time_step, x_curr_local[:, 0], label="Current X")
            ax1.plot(time_step, x_des_local[:, 0], label="Desired X")
            ax1.plot(time_step, feet_pos_des[:, 0], label="Next step X")
            ax1.set_xlabel('Time [s]')
            ax1.set_ylabel('Foot Position [m]')
            ax2.plot(time_step, x_curr_local[:, 1], label="Current Y")
            ax2.plot(time_step, x_des_local[:, 1], label="Desired Y")
            ax2.plot(time_step, feet_pos_des[:, 1], label="Next step Y")
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Foot Position [m]')
            ax3.plot(time_step, x_curr_local[:, 2], label="Current Z")
            ax3.plot(time_step, x_des_local[:, 2], label="Desired Z")
            ax3.plot(time_step, feet_pos_des[:, 2], label="Next step Z")
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Foot Position [m]')
            # ax4.plot(time_step, com_pos[:, 1], label="Current Y COM")
            # ax4.set_xlabel('Time [s]')
            # ax4.set_ylabel('Body Position [m]')
            ax1.legend()
            ax2.legend()
            ax3.legend()
            # ax4.legend()
            plt.show()

    def update_joint_states(self, msg):
        # get the correct order of joint configs/vel
        joint_config = np.zeros(12)
        joint_vel = np.zeros(12)
        joint_effort = np.zeros(12)
        for i, name in enumerate(msg.name):
            joint_config[self.joint_states_order[name]] = msg.position[i]
            joint_vel[self.joint_states_order[name]] = msg.velocity[i]
            joint_effort[self.joint_states_order[name]] = msg.effort[i]
        self.joint_config = joint_config
        self.joint_velocity = joint_vel
        self.joint_effort = joint_effort

    def update_link_states(self, msg):
        # Saving the pose
        self.robot_pose[0] = msg.pose[1].position.x
        self.robot_pose[1] = msg.pose[1].position.y
        self.robot_pose[2] = msg.pose[1].position.z
        self.robot_pose[3] = msg.pose[1].orientation.x
        self.robot_pose[4] = msg.pose[1].orientation.y
        self.robot_pose[5] = msg.pose[1].orientation.z
        self.robot_pose[6] = msg.pose[1].orientation.w
        # Saving the twist
        self.robot_twist[0] = msg.twist[1].linear.x
        self.robot_twist[1] = msg.twist[1].linear.y
        self.robot_twist[2] = msg.twist[1].linear.z
        self.robot_twist[3] = msg.twist[1].angular.x
        self.robot_twist[4] = msg.twist[1].angular.y
        self.robot_twist[5] = msg.twist[1].angular.z

        # update footprint frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'footprint'
        r = self.robot_config.get_feet_pos(np.concatenate((self.robot_pose, self.joint_config)))
        cen_x = np.mean(r[:, 0])
        cen_y = np.mean(r[:, 1])
        t.transform.translation.x = cen_x
        t.transform.translation.y = cen_y
        t.transform.translation.z = 0.0
        self.br.sendTransform(t)

        # create base_link frame for TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_pose[0]
        t.transform.translation.y = self.robot_pose[1]
        t.transform.translation.z = self.robot_pose[2]
        t.transform.rotation.x = msg.pose[1].orientation.x
        t.transform.rotation.y = msg.pose[1].orientation.y
        t.transform.rotation.z = msg.pose[1].orientation.z
        t.transform.rotation.w = msg.pose[1].orientation.w
        self.br.sendTransform(t)

    def trigger_PD_callback(self, request, response):
        self.curr_state = self.States.STAND
        return response

    def trigger_impedance_callback(self, request, response):
        self.curr_state = self.States.IMPEDANCE
        return response

    def trigger_centroidal_callback(self, request, response):
        self.curr_state = self.States.CENTROIDAL
        return response

    def trigger_reactive_planner_callback(self, request, response):
        # create a reactive planner demo class
        self.curr_state = self.States.WALK
        self.reactive_planner_demo.quadruped_dcm_reactive_stepper_start()
        return response

    def trigger_reset_effort_callback(self, request, response):
        self.curr_state = self.States.RESET_EFFORT
        return response

    def trigger_plot_callback(self, request, response):
        self.curr_state = self.States.PLOT
        return response
