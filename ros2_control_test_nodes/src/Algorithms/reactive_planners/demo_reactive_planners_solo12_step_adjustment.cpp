#include "ros2_control_test_nodes/reactive_planners/demo_reactive_planners_solo12_step_adjustment.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
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
    // kd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
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

void DemoReactivePlanner::initialize(Eigen::VectorXd &q) {
    // initialize fields for the quadruped Dcm reactive stepper
    is_left_leg_in_contact = true;
    l_min = -0.1;
    l_max = 0.1;
    w_min = -0.08;
    w_max = 0.2;
    t_min = 0.1;
    t_max = 1.0;
    l_p = 0.00;
    com_height = q(2);
    weight = Eigen::VectorXd::Zero(9);
    weight << 1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000;
    mid_air_foot_height = 0.05;
    control_period = 0.001;
    planner_loop = 0.010;

    pinocchio::framesForwardKinematics(model, data, q);

    Eigen::VectorXd base_pose = q.head(7);
    Eigen::Vector3d front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    Eigen::Vector3d front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    Eigen::Vector3d hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    Eigen::Vector3d hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    v_des = {0.0, 0.0, 0.0};
    v_des(0) = 0.2; // forward
    // y_des = 0.2; // turning
    y_des = 0.0; // forward

    // centroidal initialization
    data.M.fill(0);
    pinocchio::crba(model, data, q);
    Eigen::Vector3d inertia = data.M.block<3,3>(3,3).diagonal();
    std::cout << "Here!" << std::endl;
    std::cout << "inertia = " << inertia << std::endl;
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

Eigen::VectorXd DemoReactivePlanner::compute_torques(Eigen::VectorXd &q, Eigen::VectorXd &dq, float control_time) {

    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    dq << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    control_time = 0.001;

    // update pinocchio
    pinocchio::forwardKinematics(model, data, q); // TODO: check if this is redundant
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::computeCentroidalMomentum(model, data, q, dq);
    pinocchio::centerOfMass(model, data, q);
    pinocchio::updateFramePlacements(model, data); // TODO: check if this is redundant


    // get x_com and xd_com
    Eigen::Matrix<double, 3, 1, 0> x_com = data.com[0];
    Eigen::MatrixXd xd_com = data.vcom[0];

    // make solo go forward
//    com_des(0) = q(0) + v_des(0) * 0.001;
    double x_com_des = q(0) + v_des(0) * 0.001;
    Eigen::Vector3d com_des = {x_com_des, 0.0, com_height};
    yaw_des = 0.0;
    // make solo turn
//     yaw_des = yaw(q);
    // yaw_des = yaw_des + (y_des * 0.001);

    // get feet position
    Eigen::Vector3d front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    Eigen::Vector3d front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    Eigen::Vector3d hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    Eigen::Vector3d hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

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
//    std::cout << "com_position = " << x_com << std::endl;
//    std::cout << "com_velocity = " << xd_com << std::endl;
//    std::cout << "base_yaw = " << yaw(q) << std::endl;
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

//    std::cout << "####################################" << std::endl;
//    std::cout << "control_time = " << control_time << std::endl;
//    std::cout << "front_left_foot_position = " << front_left_foot_position << std::endl;
//    std::cout << "x_com = " << x_com << std::endl;
//    std::cout << "xd_com = " << xd_com << std::endl;
//    std::cout << "yaw(q) = " << yaw(q) << std::endl;
//    std::cout << "!open_loop = " << !open_loop << std::endl;
//    std::cout << "####################################" << std::endl;

    Eigen::VectorXd x_des_local = Eigen::VectorXd::Zero(12);
    x_des_local <<
            quadruped_dcm_reactive_stepper.get_front_left_foot_position()(0),
            quadruped_dcm_reactive_stepper.get_front_left_foot_position()(1),
            quadruped_dcm_reactive_stepper.get_front_left_foot_position()(2),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position()(0),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position()(1),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position()(2),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position()(0),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position()(1),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position()(2),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position()(0),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position()(1),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position()(2);

    Eigen::Vector4d contact_array = quadruped_dcm_reactive_stepper.get_contact_array(); // cnt_array

    for (int j = 0; j < 4; j++) {
        mim_control::ImpedanceController imp = imp_ctrls[j];
        Eigen::Vector3d foot_des_local = data.oMf[imp.get_rootframe_index()].translation();
        x_des_local.segment(3 * j, 3) = x_des_local.segment(3 * j, 3) - foot_des_local;
    }

//    std::cout << "1. x_des_local = " << x_des_local << std::endl;

    // compute w_com and F
    // convert rpy to quaternion
    Eigen::Matrix<float, 3, 3> rpy_des = pinocchio::rpy::rpyToMatrix<float>(0.0, 0.0, yaw_des);
    Eigen::Quaternionf x_ori_quat;
    x_ori_quat = rpy_des;
    Eigen::Vector4d x_ori = {x_ori_quat.x(), x_ori_quat.y(), x_ori_quat.z(), x_ori_quat.w()};
    Eigen::Vector3d x_angvel = {0.0, 0.0, y_des};

    std::cout << "kc = " << kc << std::endl;
    std::cout << "dc = " << kc << std::endl;
    std::cout << "kb = " << kc << std::endl;
    std::cout << "db = " << kc << std::endl;
    std::cout << "q.head(3) = " << q.head(3) << std::endl;
    std::cout << "dq.head(3) = " << dq.head(3) << std::endl;
    std::cout << "orientation = " << q.segment(3, 4) << std::endl;
    std::cout << "angular velocity = " << dq.segment(3, 3) << std::endl;
    std::cout << "com_des = " << com_des << std::endl;
    std::cout << "vcom_des = " << v_des << std::endl;
    std::cout << "orientation_des = " << x_ori << std::endl;
    std::cout << "angular_velocity_des = " << x_angvel << std::endl;
    // compute w_com
    centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q.head(3),
            com_des,
            dq.head(3),
            v_des,
            q.segment(3, 4),
            x_ori,
            dq.segment(3, 3),
            x_angvel
    );
     Eigen::VectorXd w_com = Eigen::VectorXd::Zero(6);
     w_com(2) = w_com(2) + 9.81 * 2.5;
     w_com = w_com + centrl_pd_ctrl.get_wrench();
//     std::cout << "2. w_com = " << w_com << std::endl;

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
//    std::cout << "3. ee_forces = " << ee_forces << std::endl;

    // get des_vel
    Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(12);
    des_vel
            << quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity();

//    std::cout << "4. des_vel = " << des_vel << std::endl;

    // passing forces to the impedance controller
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

//    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    dq << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
//    x_des_local << -0.000585, 0.04338, -0.174779, -0.00054, -0.045733, -0.17542, 0.0005263, 0.0443874, -0.174771, 0.0004180, -0.046741, -0.175435;
//    des_vel << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    ee_forces << 0.53, 0.6, 11.74, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.42, 9.76, 12.78;

    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d desired_pos = x_des_local.segment(3 * i, 3);
        pinocchio::Motion xd_des = pinocchio::Motion(des_vel.segment(3 * i, 3), Eigen::Vector3d({0, 0, 0}));
        Eigen::VectorXd kp_array(6);
        Eigen::VectorXd kd_array(6);

//        kp_array << kp.segment(3*i, 3), 0, 0, 0;
//        kd_array << kd.segment(3*i, 3), 0, 0, 0;

        if (contact_array(i) == 1) {
            kp_array = Eigen::VectorXd::Zero(6);
            kd_array = Eigen::VectorXd::Zero(6);
        } else {
            kp_array << kp.segment(3*i, 3), 0, 0, 0;
            kd_array << kd.segment(3*i, 3), 0, 0, 0;
        }
        imp_ctrls[i].run(
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

    if (control_time < 0.01) {
//        std::cout << "#####################################" << std::endl;
//        std::cout << "com_des = " << com_des << std::endl;
//        std::cout << "orientation_desired = " << x_ori << std::endl;
//        std::cout << "x_des_local = " << x_des_local << std::endl;
//        std::cout << "w_com = " << w_com << std::endl;
//        std::cout << "ee_forces = " << ee_forces << std::endl;
//        std::cout << "des_vel = " << des_vel << std::endl;
//        std::cout << "tau = " << tau << std::endl;
//        std::cout << "#####################################" << std::endl;
//        std::cout << "v_des = " << v_des << std::endl;
//        std::cout << "y_des = " << y_des << std::endl;
//        std::cout << "com_des = " << com_des << std::endl;
//        std::cout << "yaw_des = " << yaw_des << std::endl;
//        std::cout << "yaw(q) = " << yaw(q);
    }
//    std::cout << "5. tau = " << tau << std::endl;

    return tau;
}

// TODO: verify if this is correct
double DemoReactivePlanner::yaw(Eigen::VectorXd &q) {
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