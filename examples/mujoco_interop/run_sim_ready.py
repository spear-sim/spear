#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mujoco.viewer
import os

import numpy as np
import spear


def is_aabb_collide_v2(a, b, extra_extend=np.array([0, 0, 0]), debug=False):
    a_min = a[0] - a[1] - extra_extend
    a_max = a[0] + a[1] + extra_extend
    b_min = b[0] - b[1] - extra_extend
    b_max = b[0] + b[1] + extra_extend
    if a_max[0] < b_min[0] or b_max[0] < a_min[0]:
        return False
    if a_max[1] < b_min[1] or b_max[1] < a_min[1]:
        return False
    if a_max[2] < b_min[2] or b_max[2] < a_min[2]:
        return False
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf_file", default=r"F:\intel\interiorsim\pipeline\apartment_0000\mujoco_scene\main_test.mjcf")
    args = parser.parse_args()

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    mj_bodies0 = {mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if True}

    # clustering
    mj_bodies = list()
    walls = set()
    floors = set()
    ceilings = set()

    for mj_body_id in range(mj_model.nbody):
        mj_body = mj_model.body(mj_body_id)
        mj_body_name = mj_model.body(mj_body_id).name
        mj_body_name_list = mj_body_name.split("/")

        is_fixed = False

        if len(mj_body_name_list) > 2:
            actor_type = mj_body_name.split("/")[1]
            if actor_type.endswith("_wall"):
                walls.add(mj_body_name)
                is_fixed = True
            if actor_type.endswith("_floor"):
                floors.add(mj_body_name)
                is_fixed = True
            if actor_type.endswith("_ceiling"):
                floors.add(mj_body_name)
                is_fixed = True
        else:
            is_fixed = True
        aabb = mj_model.geom_aabb[mj_body_id]
        if mj_body.dofnum[0] == 6:
            movable = True
        else:
            movable = False
        pos = mj_data.body(mj_body_id).xpos
        quat = mj_data.body(mj_body_id).xquat
        mj_body_data = {
            "name": mj_body_name,
            "id": mj_body_id,
            "movable": movable,
            "is_fixed": is_fixed,
            "aabb": aabb.reshape((2, 3)),
            "location": pos,
            "rotation": quat

        }
        print(mj_body_data)
        mj_bodies.append(mj_body_data)

    edges = []
    actor_adjacent_actors = {}
    for i in range(0,len(mj_bodies)):
        actor_adjacent_actors[i] = list()
    for i in range(0, len(mj_bodies)):
        a = mj_bodies[i]
        if a['is_fixed']:
            continue
        for j in range(i + 1, len(mj_bodies)):
            b = mj_bodies[j]
            if b['is_fixed']:
                continue
            if is_aabb_collide_v2(a['aabb'], b['aabb'], debug=False):
                edges.append((i, j))
                actor_adjacent_actors[i].append(j)
                actor_adjacent_actors[j].append(i)

    # print(mj_bodies)
    print(actor_adjacent_actors)

    spear.log("Done.")
