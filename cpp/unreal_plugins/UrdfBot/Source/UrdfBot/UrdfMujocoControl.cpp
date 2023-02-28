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

    Eigen::VectorXd qfrc_applied(m->nv);
    mju_copy(qfrc_applied.data(), d->qfrc_inverse, m->nv);

    return qfrc_applied.cast<float>();
}

Eigen::VectorXf UrdfMujocoControl::task_space_control(FTransform goal_pose, FTransform eef_pose, FVector velocity, FVector angular_velocity, Eigen::VectorXf qpos, Eigen::VectorXf qvel)
{
    int eef_id = m->nbody - 1;
    float kp = 100;
    float kd = kp;
    double cm_to_m = 0.01;

    // update mj_model state
    Eigen::VectorXd qpos_d = qpos.cast<double>();
    Eigen::VectorXd qvel_d = qvel.cast<double>();
    mju_copy(d->qpos, qpos_d.data(), m->nv);
    mju_copy(d->qvel, qvel_d.data(), m->nv);
    mju_zero(d->act, m->na);
    mju_zero(d->ctrl, m->nu);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->xfrc_applied, 6 * m->nbody);
    mj_forward(m, d);

    // TODO find eef pose from qpos and check if correct?

    // find target
    Eigen::Vector3d target_location = toEigen(goal_pose.GetLocation() * cm_to_m);
    Eigen::Vector3d eef_location = toEigen(eef_pose.GetLocation() * cm_to_m);
    Eigen::Vector3d position_velocity_error(velocity.X * cm_to_m, -velocity.Y * cm_to_m, velocity.Z * cm_to_m);
    Eigen::Vector3d angular_velocity_error(angular_velocity.X, angular_velocity.Y, -angular_velocity.Z);

    Eigen::Vector3d desired_force = 1.0 * kp * (target_location - eef_location) - 1.0 * kd * position_velocity_error;
    Eigen::Vector3d desired_torque = 1.0 * -kd * angular_velocity_error;

    // compute jacobian
    Eigen::MatrixXd jac_pos(m->nv, 3);
    Eigen::MatrixXd jac_rot(m->nv, 3);
    mj_jacGeom(m, d, jac_pos.data(), jac_rot.data(), eef_id);
    Eigen::MatrixXd J_full_T(jac_pos.rows(), jac_pos.cols() + jac_rot.cols());
    J_full_T << jac_pos, jac_rot;

    Eigen::MatrixXd mass_matrix(m->nv, m->nv);
    mj_fullM(m, mass_matrix.data(), d->qM);
    Eigen::MatrixXd mass_matrix_inv = mass_matrix.completeOrthogonalDecomposition().pseudoInverse().eval();

    Eigen::MatrixXd lambda_pos_inv = jac_pos.transpose() * mass_matrix_inv * jac_pos;
    lambda_pos_inv = (lambda_pos_inv.array() < 1e-6).select(0, lambda_pos_inv);
    Eigen::MatrixXd lambda_pos = lambda_pos_inv.completeOrthogonalDecomposition().pseudoInverse().eval();

    Eigen::MatrixXd lambda_rot_inv = jac_rot.transpose() * mass_matrix_inv * jac_rot;
    lambda_rot_inv = (lambda_rot_inv.array() < 1e-6).select(0, lambda_rot_inv);
    Eigen::MatrixXd lambda_rot = lambda_rot_inv.completeOrthogonalDecomposition().pseudoInverse().eval();

    Eigen::VectorXd desired_wrench(6);
    desired_wrench.head<3>() = lambda_pos * desired_force;
    desired_wrench.tail<3>() = lambda_rot * desired_torque;

    Eigen::VectorXd torques = J_full_T * desired_wrench;

    // printMatrix("mass_matrix", mass_matrix);
    // printMatrix("lambda_pos", lambda_pos);
    // printMatrix("lambda_rot", lambda_rot);
    // printMatrix("J_full_T", J_full_T);
    printMatrix("desired_force", desired_force);
    printMatrix("desired_torque", desired_torque);
    printMatrix("torques", torques);
    return torques.cast<float>();
}

Eigen::Vector3d UrdfMujocoControl::toEigen(FVector vector)
{
    return Eigen::Vector3d(vector.X, vector.Y, vector.Z);
}

void UrdfMujocoControl::printMatrix(std::string name, Eigen::MatrixXd data)
{
    std::stringstream ss;
    ss << name << " - " << data.rows() << " " << data.cols() << std::endl;
    for (int i = 0; i < data.rows(); i++) {
        for (int j = 0; j < data.cols(); j++) {
            ss << " " << data(i, j);
        }
        if (data.cols() > 1) {
            ss << std::endl;
        }
    }
    ss << "-----------" << std::endl;

    UE_LOG(LogTemp, Log, TEXT("UrdfMujocoControl::printMatrix %s"), *FString(ss.str().c_str()));
}

#if 0
    // Compute nullspace matrix (I - Jbar * J) and lambda matrices ((J * M^-1 * J^T)^-1)

    // Gamma (without null torques) = J^T * F + gravity compensations

    // Calculate and add nullspace torques (nullspace_matrix^T * Gamma_null) to final torques
    // Note: Gamma_null = desired nullspace pose torques, assumed to be positional joint control relative
    //                     to the initial joint positions
#endif