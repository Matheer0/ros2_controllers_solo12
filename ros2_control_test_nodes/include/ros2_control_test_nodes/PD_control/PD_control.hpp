//
// Created by stanley on 6/9/22.
//
#include "pinocchio/multibody/model.hpp"

#ifndef ROS2_CONTROL_TEST_NODES_ALGORITHMS_HPP
#define ROS2_CONTROL_TEST_NODES_ALGORITHMS_HPP

#endif //ROS2_CONTROL_TEST_NODES_ALGORITHMS_HPP

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

class PD_control {
public:

    PD_control();

    PD_control(std::string &path_to_urdf);

//    void build_model(std::string &path_to_urdf);

    void set_desired_config(Eigen::Matrix<double, 12, 1> &desired_config);

    void set_params(int k_attr, double delta_t, int k, int d);

    Eigen::MatrixXd get_mass_matrix(Eigen::VectorXd &q, Eigen::VectorXd &v);

    Eigen::VectorXd get_h(Eigen::VectorXd &q, Eigen::VectorXd &v);

    Eigen::VectorXd compute_ref_vel(Eigen::VectorXd &pos_fbk);

    Eigen::VectorXd compute_ref_pos(Eigen::VectorXd &pos_fbk);

    Eigen::VectorXd
    compute_torques(Eigen::MatrixXd &m, Eigen::VectorXd &h, Eigen::VectorXd &q_dot_ref, Eigen::VectorXd &q_dot_fbk,
                    Eigen::VectorXd &q_ref, Eigen::VectorXd &q_fbk);

private:
    // pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

    // potential field planner
    int k_attr;
    double delta_t;
    Eigen::Matrix<double, 12, 1> desired_config;

    // PD control
    int k;
    int d;
};