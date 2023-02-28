//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfMujocoControl.h"

UrdfMujocoControl::UrdfMujocoControl(std::string filename)
{
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(filename.c_str(), 0, error, 1000);
    d = mj_makeData(m);

    eef_id = m->ngeom - 1;
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

Eigen::VectorXf UrdfMujocoControl::taskSpaceControl(FTransform goal_pose, FTransform eef_pose, FVector eef_velocity, FVector eef_angular_velocity, Eigen::VectorXf qpos, Eigen::VectorXf qvel)
{
    double cm_to_m = 0.01;

    // update mj_model state
    Eigen::VectorXd qpos_d = qpos.cast<double>();
    Eigen::VectorXd qvel_d = qvel.cast<double>();
    mju_copy(d->qpos, qpos_d.data(), m->nv);
    // mju_copy(d->qvel, qvel_d.data(), m->nv);
    mju_zero(d->qvel, m->nv);
    mju_zero(d->act, m->na);
    mju_zero(d->ctrl, m->nu);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->xfrc_applied, 6 * m->nbody);
    mj_forward(m, d);

    FVector desired_force = kp * (goal_pose.GetLocation() * cm_to_m - eef_pose.GetLocation() * cm_to_m) - kd * eef_velocity * cm_to_m;
    FVector desired_torque = -kd * eef_angular_velocity;

    // get jacobian
    Eigen::MatrixXd jac_pos_t(m->nv, 3);
    Eigen::MatrixXd jac_rot_t(m->nv, 3);
    mj_jacGeom(m, d, jac_pos_t.data(), jac_rot_t.data(), eef_id);
    Eigen::MatrixXd jac_full_t(jac_pos_t.rows(), jac_pos_t.cols() + jac_rot_t.cols());
    jac_full_t << jac_pos_t, jac_rot_t;

    // get mass matrix
    Eigen::MatrixXd mass_matrix(m->nv, m->nv);
    mj_fullM(m, mass_matrix.data(), d->qM);
    Eigen::MatrixXd mass_matrix_inv = mass_matrix.completeOrthogonalDecomposition().pseudoInverse().eval();

    // lambda = (J * M^-1 * J^T)^-1
    Eigen::MatrixXd lambda_pos_inv = jac_pos_t.transpose() * mass_matrix_inv * jac_pos_t;
    Eigen::MatrixXd lambda_pos = lambda_pos_inv.completeOrthogonalDecomposition().pseudoInverse().eval();
    Eigen::MatrixXd lambda_rot_inv = jac_rot_t.transpose() * mass_matrix_inv * jac_rot_t;
    Eigen::MatrixXd lambda_rot = lambda_rot_inv.completeOrthogonalDecomposition().pseudoInverse().eval();

    Eigen::VectorXd desired_wrench(6);
    desired_wrench << lambda_pos * toEigen(desired_force), lambda_rot * toEigen(desired_torque);

    Eigen::VectorXd torques = jac_full_t * desired_wrench;

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
    UE_LOG(LogTemp, Log, TEXT("[UrdfMujocoControl::printMatrix] %s"), *FString(ss.str().c_str()));
}
