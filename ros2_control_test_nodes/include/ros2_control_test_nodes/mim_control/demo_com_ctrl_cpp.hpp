#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include "ros2_control_test_nodes/mim_control/impedance_controller.hpp"
#include "ros2_control_test_nodes/mim_control/centroidal_pd_controller.hpp"
#include "ros2_control_test_nodes/mim_control/centroidal_force_qp_controller.hpp"

class DemoComCtrl {
public:
    DemoComCtrl();
    DemoComCtrl(std::string path_to_urdf);

    Eigen::VectorXd compute_torques(Eigen::VectorXd &q, Eigen::VectorXd &dq);

private:
    double mu;
    Eigen::Vector3d kc;
    Eigen::Vector3d dc;
    Eigen::Vector3d kb;
    Eigen::Vector3d db;
    Eigen::VectorXd qp_penalty_weights;

    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    Eigen::VectorXd x_des;
    pinocchio::Motion xd_des;

    std::vector<std::string> endeff_names;

    pinocchio::Model model;
    pinocchio::Data data;

    std::vector<mim_control::ImpedanceController> imp_ctrls;

    mim_control::CentroidalPDController centrl_pd_ctrl;
    mim_control::CentroidalForceQPController force_qp;

    Eigen::Vector3d x_com;
    Eigen::Vector3d xd_com;
    Eigen::Vector4d x_ori;
    Eigen::Vector3d x_angvel;
    Eigen::Vector4d cnt_array;
};
