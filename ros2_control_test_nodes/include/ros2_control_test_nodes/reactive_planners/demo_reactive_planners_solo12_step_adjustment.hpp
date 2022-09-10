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

    void initialize(Eigen::VectorXd &q);

    Eigen::VectorXd compute_torques(Eigen::VectorXd &q, Eigen::VectorXd &dq, float control_time);

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
    float mu;
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
    float l_min;
    float l_max;
    float w_min;
    float w_max;
    float t_min;
    float t_max;
    float l_p;
    double com_height;
    Eigen::VectorXd weight;
    float mid_air_foot_height;
    float control_period;
    float planner_loop;
    reactive_planners::QuadrupedDcmReactiveStepper quadruped_dcm_reactive_stepper;

    // more fields
    Eigen::Vector3d v_des;
    float y_des; // speed of yaw angle
    // Eigen::Vector3d x_com; // TODO: I used a vector here but they used a matrix instead
    double yaw_des;
    Eigen::Vector2d cnt_array;
    bool open_loop;
    Eigen::Vector3d dcm_force;

    // helper methods
    static double yaw(Eigen::VectorXd &q);
};