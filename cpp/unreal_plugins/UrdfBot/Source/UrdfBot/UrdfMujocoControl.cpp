//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfMujocoControl.h"

#include <iostream>

UrdfMujocoControl::UrdfMujocoControl()
{
    char error[1000] = "Could not load binary model";
    std::string model_filename = "E:/intel/interiorsim/python/spear/urdf/double_pendulum_mujoco.xml";
    m = mj_loadXML(model_filename.c_str(), 0, error, 1000);
    d = mj_makeData(m);
}

UrdfMujocoControl::~UrdfMujocoControl()
{
    mj_deleteData(d);
    mj_deleteModel(m);
}

void UrdfMujocoControl::test()
{
}

std::vector<float> UrdfMujocoControl::get_qfrc_inverse(std::vector<float> qpos)
{
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qvel, m->nv);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->ctrl, m->nv);

    std::vector<double> qpos_d;
    for (int i = 0; i < m->nv; i++) {
        qpos_d.push_back(qpos[i]);
    }

    mju_copy(d->qpos, qpos_d.data(), m->nv);

    mj_inverse(m, d);

    std::vector<float> result;
    for (int i = 0; i < m->nv;i++) {
        result.push_back(d->qfrc_inverse[i]);
    }
    return result;
}
