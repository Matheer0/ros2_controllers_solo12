//
// Created by stanley on 6/9/22.
//
#include "ros2_control_test_nodes/PD_control/PD_control.hpp"

PD_control::PD_control() {}

PD_control::PD_control(std::string &path_to_urdf) {
    pinocchio::urdf::buildModel(path_to_urdf, model);
    data = pinocchio::Data(model);
}

//void PD_control::build_model(std::string &path_to_urdf) {
//        pinocchio::urdf::buildModel(path_to_urdf, model);
//        data = pinocchio::Data(model);
//}

void PD_control::set_desired_config(Eigen::Matrix<double, 12, 1> &desired_config_param) {
    this -> desired_config = desired_config_param;
};

void PD_control::set_params(int k_attr, double delta_t, int k, int d) {
    this -> k_attr = k_attr;
    this -> delta_t = delta_t;
    this -> k = k;
    this -> d = d;
}

// pinocchio functions
Eigen::MatrixXd PD_control::get_mass_matrix(Eigen::VectorXd &q, Eigen::VectorXd &v) {
    return pinocchio::crba(model, data, q);

}

Eigen::VectorXd PD_control::get_h(Eigen::VectorXd &q, Eigen::VectorXd &v) {
    Eigen::VectorXd aq0 = Eigen::VectorXd::Zero(model.nv);
    return pinocchio::rnea(model, data, q, v, aq0);
}

// potential field planner functions
Eigen::VectorXd PD_control::compute_ref_vel(Eigen::VectorXd &pos_fbk) {
    return k_attr * (desired_config - pos_fbk);
}

Eigen::VectorXd PD_control::compute_ref_pos(Eigen::VectorXd &pos_fbk) {
    return pos_fbk + compute_ref_vel(pos_fbk) * delta_t;
}

// PD control
Eigen::VectorXd
PD_control::compute_torques(Eigen::MatrixXd &m,
                            Eigen::VectorXd &h,
                            Eigen::VectorXd &q_dot_ref,
                            Eigen::VectorXd &q_dot_fbk,
                            Eigen::VectorXd &q_ref,
                            Eigen::VectorXd &q_fbk) {
    Eigen::MatrixXd q_ddot_target = d * (q_dot_ref - q_dot_fbk) + k * (q_ref - q_fbk);
    // Eigen::VectorXd tau = m * q_ddot_target + h;
    // std::cout << "num of rows: " << tau.rows() << ", num of cols: " << tau.cols() << std::endl;
    return m * q_ddot_target + h;
}
