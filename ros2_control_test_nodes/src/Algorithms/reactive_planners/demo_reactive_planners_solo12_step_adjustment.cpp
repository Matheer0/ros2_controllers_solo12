#include "ros2_control_test_nodes/reactive_planners/demo_reactive_planners_solo12_step_adjustment.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/math/rpy.hpp>
#include <cmath>

DemoReactivePlanner::DemoReactivePlanner() {}

DemoReactivePlanner::DemoReactivePlanner(std::string path_to_urdf) {
    // building the pinocchio model
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(path_to_urdf, root_joint, model);
    data = pinocchio::Data(model);

    mu = 0.6;
    kc = {0.0, 0.0, 200.0};
    dc = {10.0, 10.0, 10.0};
    kb = {25.0, 25.0, 25.0};
    db = {22.5, 22.5, 22.5};
    qp_penalty_lin = {1e0, 1e0, 1e6};
    qp_penalty_ang = {1e6, 1e6, 1e6};
    Eigen::VectorXd qp_penalty_weights(qp_penalty_lin.size() + qp_penalty_ang.size());
    qp_penalty_weights << qp_penalty_lin, qp_penalty_ang;

    centrl_pd_ctrl = mim_control::CentroidalPDController();

    force_qp = mim_control::CentroidalForceQPController();
    force_qp.initialize(4, mu, qp_penalty_weights);

    // initialize impedance controllers
    kp = Eigen::VectorXd::Zero(12);
    kd = Eigen::VectorXd::Zero(12);
    kp << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    kd << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    std::vector<std::string> frame_root_names = {"FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"};
    endeff_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};
    imp_ctrls = {mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController()};
    for (int i = 0; i < 4; i++) {
        imp_ctrls[i].initialize(model, frame_root_names[i], endeff_names[i]);
    }

    quadruped_dcm_reactive_stepper = reactive_planners::QuadrupedDcmReactiveStepper();
}

void DemoReactivePlanner::initialize(Eigen::Matrix<double, 19, 1> &q) {
    // initialize fields for the quadruped Dcm reactive stepper
    is_left_leg_in_contact = true;
    l_min = -0.1;
    l_max = 0.1;
    w_min = -0.08;
    w_max = 0.2;
    t_min = 0.12;
    t_max = 2.0;
    l_p = 0.00;
    com_height = q(2);
    weight = Eigen::VectorXd::Zero(9);
    weight << 1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000;
    mid_air_foot_height = 0.05;
    control_period = 0.001;
    planner_loop = 0.010;

    pinocchio::framesForwardKinematics(model, data, q);

    Eigen::VectorXd base_pose = q.head(7);
    front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    v_des = {0.0, 0.0, 0.0};
//    v_des(0) = 0.2; // forward
//    y_des = 0.0; // forward
    y_des = 0.2; // turning

    // centroidal initialization
    data.M.fill(0);
    pinocchio::crba(model, data, q);
    Eigen::Vector3d inertia = data.M.block<3, 3>(3, 3).diagonal();
//    inertia = {0.04196225, 0.0699186, 0.08607027};
    centrl_pd_ctrl.initialize(2.5, inertia);

    // initialize quadruped_dcm_reactive_stepper
    quadruped_dcm_reactive_stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
            base_pose,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position
    );

    quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des);
    quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory();

    // initialize more fields
    yaw_des = yaw(q);
    cnt_array = {0.0, 0.0};
    open_loop = true;
    dcm_force = {0.0, 0.0, 0.0};
}

Eigen::VectorXd DemoReactivePlanner::compute_torques(Eigen::Matrix<double, 19, 1> &q, Eigen::Matrix<double, 18, 1> &dq,
                                                     double control_time) {
    // update pinocchio
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::computeCentroidalMomentum(model, data, q, dq);

    // get x_com and xd_com
    Eigen::Matrix<double, 3, 1, 0> x_com = data.com[0];
    Eigen::MatrixXd xd_com = data.vcom[0];

    // make solo go forward
//    double x_com_des = q(0) + v_des(0) * 0.001;
//    Eigen::Vector3d com_des = {x_com_des, 0.0, com_height};
//    yaw_des = 0.0;
    // make solo turn
    v_des(0) = 0.0;
    y_des = 0.2;
    // yaw_des = yaw_des + y_des * 0.001;
    yaw_des = yaw(q);
    yaw_des = yaw_des + (y_des * 0.001);
//    std::cout << "yaw_des (degrees) = " << (yaw_des * 180 / M_PI) << std::endl;
    Eigen::Vector3d com_des = {0.0, 0.0, com_height};
    com_des = {q[0], q[1], com_height};

    // get feet position
    front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    // get feet velocity
    Eigen::Vector3d front_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[0].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d front_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                            imp_ctrls[1].get_endframe_index(),
                                                                            pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                          imp_ctrls[2].get_endframe_index(),
                                                                          pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[3].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();
    // compute x_des local
    quadruped_dcm_reactive_stepper.run(
            control_time,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
            front_left_foot_velocity,
            front_right_foot_velocity,
            hind_left_foot_velocity,
            hind_right_foot_velocity,
            x_com,
            xd_com,
            yaw(q),
            !open_loop
    );

    x_des_local <<
                quadruped_dcm_reactive_stepper.get_front_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position();

    Eigen::Vector4d contact_array = quadruped_dcm_reactive_stepper.get_contact_array(); // cnt_array

    for (int j = 0; j < 4; j++) {
        foot_des_local = data.oMf[imp_ctrls[j].get_rootframe_index()].translation();
        x_des_local.segment(3 * j, 3) = x_des_local.segment(3 * j, 3) - foot_des_local;
    }

    double roll = 0.0;
    double pitch = 0.0;
    Eigen::Quaterniond x_ori;
    x_ori = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(yaw_des, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d x_angvel = {0.0, 0.0, y_des / 2};

    Eigen::Quaterniond curr_orientation = Eigen::Quaterniond(q(6), q(5), q(4), q(3));
    curr_orientation.normalize();
    Eigen::MatrixXd curr_rot = curr_orientation.toRotationMatrix();

    auto extract_yaw = [](Eigen::Vector4d quat) {
        double x = quat(0);
        double y = quat(1);
        double z = quat(2);
        double w = quat(3);
        return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
    };

    // compute w_com
    centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q.head(3),
            com_des,
            curr_rot * dq.head(3),
            v_des,
            q.segment(3, 4),
            x_ori.coeffs(),
            dq.segment(3, 3),
            x_angvel
    );
//    std::cout << "q.head(3) = " << q.head(3) << std::endl;
//    std::cout << "dq.head(3) = " << dq.head(3) << std::endl;
//    std::cout << "orientation = " << q.segment(3, 4) << std::endl;
//    std::cout << "angular velocity = " << dq.segment(3, 3) << std::endl;
//    std::cout << "com_des = " << com_des << std::endl;
//    std::cout << "vcom_des = " << v_des << std::endl;
//    std::cout << "orientation_des = " << x_ori.coeffs() << std::endl;
//    std::cout << "angular_velocity_des = " << x_angvel << std::endl;
//    std::cout << "orientation = " << x_ori.coeffs() << std::endl;
//    std::cout << "current yaw = " << (yaw(q) * 180 / M_PI) << std::endl;
//    std::cout << "yaw_des (degrees) = " << (yaw_des * 180 / M_PI) << std::endl;

    Eigen::VectorXd w_com = Eigen::VectorXd::Zero(6);
    w_com(2) = w_com(2) + 9.81 * 2.5;
    w_com = w_com + centrl_pd_ctrl.get_wrench();

    // compute ee_forces
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

    force_qp.run(w_com, rel_eff, contact_array);
    Eigen::VectorXd ee_forces = force_qp.get_forces();

    // get des_vel
    Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(12);
    des_vel << quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity();

    // passing forces to the impedance controller
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

    for (int i = 0; i < 4; i++) {
        desired_pos = x_des_local.segment(3 * i, 3);
        xd_des = pinocchio::Motion(des_vel.segment(3 * i, 3), Eigen::Vector3d({0, 0, 0}));

        if (contact_array(i) == 1) {
            kp_array = Eigen::VectorXd::Zero(6);
            kd_array = Eigen::VectorXd::Zero(6);
        } else {
            kp_array << kp.segment(3 * i, 3), 0, 0, 0;
            kd_array << kd.segment(3 * i, 3), 0, 0, 0;
        }
        imp_ctrls[i].run_local(
                q,
                dq,
                kp_array.array(),
                kd_array.array(),
                1.0,
                pinocchio::SE3(Eigen::Matrix3d::Identity(), desired_pos),
                xd_des,
                pinocchio::Force(ee_forces.segment(3 * i, 3), Eigen::Vector3d::Zero(3))
        );
        tau = tau + imp_ctrls[i].get_joint_torques();
    }

    return tau;
}

double DemoReactivePlanner::yaw(Eigen::Matrix<double, 19, 1> &q) {
    Eigen::Vector4d quat = q.segment(3, 4);
    double x = quat(0);
    double y = quat(1);
    double z = quat(2);
    double w = quat(3);
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

void DemoReactivePlanner::quadruped_dcm_reactive_stepper_start() {
    quadruped_dcm_reactive_stepper.start();
}