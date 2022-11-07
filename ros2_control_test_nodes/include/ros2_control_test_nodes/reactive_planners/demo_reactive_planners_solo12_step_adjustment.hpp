//
// Created by stanley on 7/5/22.
//

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include "ros2_control_test_nodes/mim_control/impedance_controller.hpp"
#include "ros2_control_test_nodes/mim_control/centroidal_pd_controller.hpp"
#include "ros2_control_test_nodes/mim_control/centroidal_force_qp_controller.hpp"
#include "ros2_control_test_nodes/reactive_planners/quadruped_dcm_reactive_stepper.hpp"

class DemoReactivePlanner {
public:
    DemoReactivePlanner();

    DemoReactivePlanner(std::string path_to_urdf);

    void initialize(Eigen::Matrix<double, 19, 1> &q);

    Eigen::VectorXd
    compute_torques(Eigen::Matrix<double, 19, 1> &q, Eigen::Matrix<double, 18, 1> &dq, double control_time);

    void quadruped_dcm_reactive_stepper_start();

private:

    // pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

    // impedance controller
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    std::vector<std::string> endeff_names;
    std::vector<mim_control::ImpedanceController> imp_ctrls;

    // centroidal controller
    double mu;
    Eigen::Vector3d kc;
    Eigen::Vector3d dc;
    Eigen::Vector3d kb;
    Eigen::Vector3d db;
    Eigen::Vector3d qp_penalty_lin;
    Eigen::Vector3d qp_penalty_ang;
    mim_control::CentroidalPDController centrl_pd_ctrl;
    mim_control::CentroidalForceQPController force_qp;

    // Quadruped DCM reactive stepper
    bool is_left_leg_in_contact;
    double l_min;
    double l_max;
    double w_min;
    double w_max;
    double t_min;
    double t_max;
    double l_p;
    double com_height;
    Eigen::VectorXd weight;
    double mid_air_foot_height;
    double control_period;
    double planner_loop;
    reactive_planners::QuadrupedDcmReactiveStepper quadruped_dcm_reactive_stepper;

    // more fields
    Eigen::Vector3d v_des;
    double y_des; // speed of yaw angle
    double yaw_des;
    Eigen::Vector2d cnt_array;
    bool open_loop;
    Eigen::Vector3d dcm_force;

    // vectors used in the compute_torques() method
    Eigen::Vector3d front_left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d front_right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d hind_left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d hind_right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 12, 1> x_des_local = Eigen::Matrix<double, 12, 1>::Zero(12);
    mim_control::ImpedanceController imp;
    Eigen::Vector3d foot_des_local = Eigen::Vector3d::Zero();

    Eigen::Vector3d desired_pos;
    pinocchio::Motion xd_des;
    Eigen::Matrix<double, 6, 1> kp_array;
    Eigen::Matrix<double, 6, 1> kd_array;

    // helper methods
    static double yaw(Eigen::Matrix<double, 19, 1> &q);
};