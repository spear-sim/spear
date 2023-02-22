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
