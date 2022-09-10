#include "ros2_control_test_nodes/mim_control/demo_com_ctrl_cpp.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

DemoComCtrl::DemoComCtrl() {}

DemoComCtrl::DemoComCtrl(std::string path_to_urdf) {
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(path_to_urdf, root_joint, model);
    data = pinocchio::Data(model);

    mu = 0.7;
    kc = {40, 40, 40};
    dc = {20, 20, 20};
    kb = {4, 4, 4};
    db = {4, 4, 4};
    qp_penalty_weights = Eigen::VectorXd::Zero(6);
    qp_penalty_weights << 5e5, 5e5, 5e5, 1e6, 1e6, 1e6;
    // create mim_control_ controllers
    std::string root_name = "universe";
    endeff_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};
    imp_ctrls = {mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController()};
    for (int i = 0; i < 4; i++) {
        imp_ctrls[i].initialize(model, root_name, endeff_names[i]);
    }

    // create centroidal controller
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(19);
    q_init[7] = 1;
    centrl_pd_ctrl = mim_control::CentroidalPDController();
    // values below obtained from printing the 2nd argument of centrl_pd_ctrl.initialize() in demo_robot_com_ctrl_cpp.py in mim_control_ repo
    Eigen::Vector3d inertia = {0.04196225, 0.0699186, 0.08607027};
    centrl_pd_ctrl.initialize(2.5, inertia);

    force_qp = mim_control::CentroidalForceQPController();
    force_qp.initialize(4, mu, qp_penalty_weights);

    // Desired center of mass position and velocity
    x_com = {0.0, 0.0, 0.18};
    xd_com = {0.0, 0.0, 0.0};
    x_ori = {0.0, 0.0, 0.0, 1.0}; // flat base
    x_angvel = {0.0, 0.0, 0.0};
    cnt_array = {1, 1, 1, 1};

    // Impedance gains
    kp = Eigen::VectorXd::Zero(6);
    kd = Eigen::VectorXd::Zero(6);

    x_des = Eigen::VectorXd::Zero(12);
    x_des << 0.195, 0.147, 0.015, 0.195, -0.147, 0.015, -0.195, 0.147, 0.015, -0.195, -0.147, 0.015;
    // xd_des = Eigen::VectorXd::Zero(12);
    xd_des = pinocchio::Motion(Eigen::Vector3d({0, 0, 0}), Eigen::Vector3d({0, 0, 0}));
}

Eigen::VectorXd DemoComCtrl::compute_torques(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

    // computing forces to be applied in the centroidal space
    centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q.head(3),
            x_com,
            dq.head(3), // assuming mim_control_ code had dq in local frame and this dq is in world frame so no conversion needed
            xd_com,
            q.segment(3, 4),
            x_ori,
            dq.segment(3, 3),
            x_angvel
    );

    Eigen::VectorXd w_com = Eigen::VectorXd::Zero(6);
    w_com(2) = w_com(2) + 9.8 * 2.5;
    w_com = w_com + centrl_pd_ctrl.get_wrench();
    pinocchio::framesForwardKinematics(model, data, q);

    Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q);
    Eigen::VectorXd rel_eff = Eigen::VectorXd::Zero(12);
    int i = 0;
    for (const std::string &eff_name: endeff_names) {
        int id = model.getFrameId(eff_name);
        Eigen::Vector3d diff = data.oMf[id].translation() - com;
        rel_eff(i * 3) = diff(0);
        rel_eff(i * 3 + 1) = diff(1);
        rel_eff(i * 3 + 2) = diff(2);
        i++;
    }
    force_qp.run(w_com, rel_eff, cnt_array);
    Eigen::VectorXd ee_forces = force_qp.get_forces();

    // passing forces to the impedance controller
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d desired_pos = x_des.segment(3 * i, 3);
        imp_ctrls[i].run(
                q,
                dq,
                kp.array(),
                kd.array(),
                1.0,
                pinocchio::SE3(Eigen::Matrix3d::Identity(), desired_pos),
                pinocchio::Motion(xd_des),
                pinocchio::Force(ee_forces.segment(3 * i, 3), Eigen::Vector3d::Zero(3))
        );
        tau = tau + imp_ctrls[i].get_joint_torques();
    }
    return tau;
}