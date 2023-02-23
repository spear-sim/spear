//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfMujocoControl.h"

UrdfMujocoControl::UrdfMujocoControl(std::string filename)
{
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(filename.c_str(), 0, error, 1000);
    d = mj_makeData(m);
}

UrdfMujocoControl::~UrdfMujocoControl()
{
    mj_deleteData(d);
    mj_deleteModel(m);
}

Eigen::VectorXf UrdfMujocoControl::inverseDynamics(Eigen::VectorXf qpos)
{
    Eigen::VectorXd qpos_d = qpos.cast<double>();
    mju_copy(d->qpos, qpos_d.data(), m->nv);
    mju_zero(d->qvel, m->nv);
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->ctrl, m->nv);

    mj_inverse(m, d);

    Eigen::VectorXd qfrc_applied;
    qfrc_applied.resize(m->nv);
    mju_copy(qfrc_applied.data(), d->qfrc_inverse, m->nv);

    return qfrc_applied.cast<float>();
}

Eigen::VectorXf UrdfMujocoControl::task_space_control(FTransform goal_pose, FTransform eef_pose, FVector velocity, FVector angular_velocity, Eigen::VectorXf qpos, Eigen::VectorXf qvel)
{
    // update mj_model
    Eigen::VectorXd qpos_d = qpos.cast<double>();
    Eigen::VectorXd qvel_d = qpos.cast<double>();
    mju_copy(d->qpos, qpos_d.data(), m->nv);
    mju_copy(d->qpos, qvel_d.data(), m->nv);
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->ctrl, m->nv);

    mj_forward(m, d);

    float kp = 150 * 20;
    float kd = 24.495 * 20;
    FVector position_error = goal_pose.GetLocation() - eef_pose.GetLocation();
    FVector position_velocity_error = -velocity;
    FQuat orientation_error = goal_pose.GetRotation() - eef_pose.GetRotation();
    FVector angular_velocity_error = -angular_velocity;

    // F_r = kp * pos_err + kd * vel_err
    FVector desired_force = kp * position_error + kd * position_velocity_error;
    // # Tau_r = kp * ori_err + kd * vel_err
    FVector desired_torque = kp * orientation_error.Euler() + kd * angular_velocity_error;

#if 0
    // Compute nullspace matrix (I - Jbar * J) and lambda matrices ((J * M^-1 * J^T)^-1)

    // Gamma (without null torques) = J^T * F + gravity compensations

    // Calculate and add nullspace torques (nullspace_matrix^T * Gamma_null) to final torques
    // Note: Gamma_null = desired nullspace pose torques, assumed to be positional joint control relative
    //                     to the initial joint positions
#endif

    Eigen::VectorXd desired_wrench;
    desired_wrench.resize(6);
    desired_wrench(0) = desired_force.X;
    desired_wrench(1) = desired_force.Y;
    desired_wrench(2) = desired_force.Z;
    desired_wrench(3) = desired_torque.X;
    desired_wrench(4) = desired_torque.Y;
    desired_wrench(5) = desired_torque.Z;

    Eigen::VectorXd res;
    res.resize(6);
    mju_mulMatTVec(res.data(), d->efc_J, desired_wrench.data(), d->nefc, m->nv);

    return res.cast<float>();
}
