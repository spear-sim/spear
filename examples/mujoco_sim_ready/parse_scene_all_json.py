#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import sys

import mujoco.viewer
import os

import xml.etree.ElementTree as ET
import numpy as np
import spear
from scipy.spatial.transform import Rotation as R


def is_aabb_collide_v2(a_center, a_size, b_center, b_size, extra_extend=np.array([0, 0, 0]), debug=False):
    a_min = a_center - a_size - extra_extend
    a_max = a_center + a_size + extra_extend
    b_min = b_center - b_size - extra_extend
    b_max = b_center + b_size + extra_extend
    if a_max[0] < b_min[0] or b_max[0] < a_min[0]:
        return False
    if a_max[1] < b_min[1] or b_max[1] < a_min[1]:
        return False
    if a_max[2] < b_min[2] or b_max[2] < a_min[2]:
        return False
    return True


def save_scene_data(mj_bodies, filename):
    result = []

    for body in mj_bodies:
        body_new = {}
        for key, value in body.items():
            if isinstance(value, np.ndarray):
                body_new[key] = value.tolist()
            else:
                body_new[key] = value
        result.append(body_new)
    json.dump(result, open(filename, mode='w'))
    return result


def build_mj_bodies(mjcf_file, debug=False):
    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(mjcf_file)
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

        # skip sub component body
        if str(mj_body_name).count(".") > 0:
            continue

        mj_body_name_list = mj_body_name.split("/")
        if len(mj_body_name_list) >= 2:
            actor_type = mj_body_name.split("/")[1]
            if actor_type.endswith("_wall"):
                walls.add(mj_body_name)
                is_fixed = True
            elif actor_type.endswith("_floor"):
                floors.add(mj_body_name)
                is_fixed = True
            elif actor_type.endswith("_ceiling"):
                floors.add(mj_body_name)
                is_fixed = True
            else:
                is_fixed = False
        else:
            continue

        if mj_body.dofnum[0] == 6:
            movable = True
        else:
            movable = False

        pos = mj_data.body(mj_body_id).xpos
        quat = mj_data.body(mj_body_id).xquat

        # https://github.com/google-deepmind/mujoco/issues/1032
        body_min = np.array([sys.float_info.max, sys.float_info.max, sys.float_info.max])
        body_max = body_min * -1
        for geom_offset in range(0, mj_body.geomnum[0]):
            geom = mj_data.geom(mj_body.geomadr + geom_offset)
            geom_aabb = mj_model.geom_aabb[geom.id].reshape((2, 3))
            geom_center = geom.xmat.reshape((3, 3)).dot(geom_aabb[0]) + geom.xpos
            geom_size = np.abs(geom.xmat.reshape((3, 3)).dot(geom_aabb[1]))
            geom_min = geom_center - geom_size
            geom_max = geom_center + geom_size
            body_min = np.minimum(body_min, geom_min)
            body_max = np.maximum(body_max, geom_max)

        world_center = (body_max + body_min) * 0.5
        world_size = (body_max - body_min) * 0.5

        if debug:
            print(mj_body_name, pos, world_center, world_size)

        mj_body_data = {
            "name": mj_body_name,
            "id": mj_body_id,
            "movable": movable,
            "is_fixed": is_fixed,
            "world_center": world_center,
            "world_size": world_size,
            "location": pos,
            "rotation": quat
        }
        mj_bodies.append(mj_body_data)
    return mj_bodies


def build_connectivity(mj_bodies, aabb_extend, debug=False):
    extra_extend = np.array([1, 1, 1]) * aabb_extend
    edges = []
    actor_adjacent_actors = {}
    for i in range(0, len(mj_bodies)):
        actor_adjacent_actors[i] = list()

    for i in range(0, len(mj_bodies)):
        a = mj_bodies[i]
        if a['is_fixed']:
            continue
        for j in range(i + 1, len(mj_bodies)):
            b = mj_bodies[j]
            if b['is_fixed']:
                continue
            if is_aabb_collide_v2(a['world_center'], a['world_size'], b['world_center'], b['world_size'], extra_extend=extra_extend, debug=False):
                edges.append((i, j))
                actor_adjacent_actors[i].append(j)
                actor_adjacent_actors[j].append(i)

    if debug:
        for index, adjList in actor_adjacent_actors.items():
            print(mj_bodies[index]['name'])
            for adjIndex in adjList:
                print("    ", mj_bodies[adjIndex]['name'])
    return actor_adjacent_actors


def build_clusters(mj_bodies, actor_adjacent_actors, debug=False):
    cluster_map = {}
    actor_cluster = []
    for i in range(0, len(mj_bodies)):
        body = mj_bodies[i]
        if not body["is_fixed"]:
            cluster_map[i] = set()
            cluster_map[i].add(i)
            actor_cluster.append(i)
        else:
            actor_cluster.append(-1)

    for i in range(0, len(mj_bodies)):
        for j in actor_adjacent_actors[i]:
            # add all j to i
            if actor_cluster[i] != actor_cluster[j]:
                cluster_id = min(actor_cluster[i], actor_cluster[j])
                removed_cluster_id = max(actor_cluster[i], actor_cluster[j])
                for k in cluster_map[removed_cluster_id]:
                    actor_cluster[k] = cluster_id
                    cluster_map[cluster_id].add(k)

                cluster_map.pop(removed_cluster_id)
    # print("clustering complete", len(clusters))
    if debug:
        print("debug clusters")
        print("#cluster=", len(cluster_map), "#body=", len(mj_bodies))
        cluster_index = 0
        for cluster_id, cluster in cluster_map.items():
            cluster_names = []
            for index in cluster:
                cluster_names.append(mj_bodies[index]["name"])
            print(cluster_index, cluster_id, cluster_names)
            cluster_index += 1
    return list(cluster_map.values())


def get_all_bodies(filename, debug=False, bodies_dir=None):
    bodies = ET.parse(filename).getroot()
    assert bodies.tag == "bodies"

    body_elements = {}
    for body_node in bodies:
        if body_node.tag == "body":
            name = body_node.attrib["name"]
            pos = body_node.attrib["pos"]
            xyaxes = body_node.attrib["xyaxes"]
            if debug:
                print("body", name, pos, xyaxes)
            body_elements[name] = body_node

            # save body to separated xml
            if bodies_dir is not None:
                body_dir = os.path.join(bodies_dir, f"{name}.xml".replace(":", "_"))
                os.makedirs(os.path.dirname(body_dir), exist_ok=True)
                ET.ElementTree(body_node).write(body_dir)

    return body_elements


mjcf_template = """
<mujoco model="apartment_0000">
    <option gravity="0 0 -981"/>
    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
        <include file="meshes.mjcf"/>
    </asset>
    <worldbody>
        <geom size="1000 1000 1" type="plane" material="grid"/>
        <light pos="0 0 1000"/>
    </worldbody>
</mujoco>
"""


def build_mjcf(body_indexes, mj_bodies, body_elements, dst):
    # spear.log("build_mjcf init.")
    mjcf_model = ET.ElementTree(ET.fromstring(mjcf_template))

    worldbody = mjcf_model.getroot().find("worldbody")
    for body_index in body_indexes:
        mj_body = mj_bodies[body_index]
        name = mj_body["name"]
        if name in body_elements:
            worldbody.append(body_elements[name])
        else:
            pass
    mjcf_model.write(dst)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pipeline_dir", default=r"F:\intel\interiorsim\pipeline")
    parser.add_argument("--scene_id", default="apartment_0000")
    parser.add_argument("--mjcf_name", default="main")
    # parser.add_argument("--template", default=r"F:\intel\interiorsim\pipeline\apartment_0000\mujoco_scene\template.mjcf")
    args = parser.parse_args()

    # load bodies
    mj_bodies = build_mj_bodies(os.path.join(args.pipeline_dir, args.scene_id, "mujoco_scene", f"{args.mjcf_name}.mjcf"), debug=True)
    fixed_bodies = []
    for i in range(len(mj_bodies)):
        mj_body = mj_bodies[i]
        if mj_body['is_fixed']:
            fixed_bodies.append(i)
    # save_scene_data(mj_bodies, "mj_bodies.json")

    # build adjacency
    actor_adjacent_actors = build_connectivity(mj_bodies, aabb_extend=10, debug=False)

    # clustering
    clusters = build_clusters(mj_bodies, actor_adjacent_actors, debug=True)

    # export separated mjcf
    body_elements = get_all_bodies(os.path.join(args.pipeline_dir, args.scene_id, "mujoco_scene", "bodies.mjcf"))
    for cluster_index in range(len(clusters)):
        cluster = clusters[cluster_index]
        body_indexes = []
        for i in fixed_bodies:
            body_indexes.append(i)
        for i in cluster:
            body_indexes.append(i)

        build_mjcf(body_indexes, mj_bodies, body_elements, os.path.join(args.pipeline_dir, args.scene_id, "mujoco_scene", f"cluster_{cluster_index:02d}.mjcf"))

    spear.log("Done.")
