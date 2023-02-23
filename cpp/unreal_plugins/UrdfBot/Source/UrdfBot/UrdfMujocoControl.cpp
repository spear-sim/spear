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
    float kp = 3;
    float kd = 3;

    // update mj_model
    Eigen::VectorXd qpos_d = qpos.cast<double>();
    Eigen::VectorXd qvel_d = qvel.cast<double>();
    mju_copy(d->qpos, qpos_d.data(), m->nv);
    mju_copy(d->qpos, qvel_d.data(), m->nv);
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->ctrl, m->nv);

    mj_forward(m, d);

    FVector position_error = goal_pose.GetLocation() - eef_pose.GetLocation();
    FVector position_velocity_error = -velocity;
    FQuat orientation_error = goal_pose.GetRotation() - eef_pose.GetRotation();
    FVector angular_velocity_error = -angular_velocity;

    // F_r = kp * pos_err + kd * vel_err
    FVector desired_force = kp * position_error + kd * position_velocity_error;
    // # Tau_r = kp * ori_err + kd * vel_err
    FVector desired_torque = kp * orientation_error.Euler() + kd * angular_velocity_error;

    int eef_id = qpos.size() - 1;
    Eigen::VectorXd jac_pos(m->nv * 3);
    Eigen::VectorXd jac_rot(m->nv * 3);
    mj_jacGeom(m, d, jac_pos.data(), jac_rot.data(), eef_id);

    Eigen::Vector3d err(position_error.X, position_error.Y, position_error.Z);

    Eigen::VectorXd qfrc_applied(m->nv);
    mju_mulMatTMat(qfrc_applied.data(), jac_pos.data(), err.data(), 3, m->nv, 1);

    return qfrc_applied.cast<float>();
}

#if 0
    // Compute nullspace matrix (I - Jbar * J) and lambda matrices ((J * M^-1 * J^T)^-1)

    // Gamma (without null torques) = J^T * F + gravity compensations

    // Calculate and add nullspace torques (nullspace_matrix^T * Gamma_null) to final torques
    // Note: Gamma_null = desired nullspace pose torques, assumed to be positional joint control relative
    //                     to the initial joint positions
#endif