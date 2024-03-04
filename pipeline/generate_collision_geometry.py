#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import coacd
import json
import numpy as np
import os
import pathlib
import spear
import spear.pipeline
import trimesh

parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
args = parser.parse_args()

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]
physics_constraint_component_classes = ["PhysicsConstraintComponent"]


def process_scene():

    kinematic_trees_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "kinematic_trees"))
    assert os.path.exists(kinematic_trees_dir)
    actors_json_file = os.path.realpath(os.path.join(kinematic_trees_dir, "actors.json"))
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    actors = actors_json.items()

    for actor_name, actor_kinematic_tree in actors:
        if actor_name != "Meshes/03_cabinet/Cabinet":
            continue
        spear.log("Processing actor: ", actor_name)
        generate_collision_geometry(actor_name=actor_name, node=actor_kinematic_tree["root_node"], log_prefix_str="    ")

    spear.log("Done.")


def generate_collision_geometry(actor_name, node, log_prefix_str):

    spear.log(log_prefix_str, "Processing kinematic tree node: ", node["name"])

    # TODO: get merge_id from the static mesh data
    merge_id = 0
    static_mesh_components_unmerged = node["static_mesh_components"].items()
    static_mesh_components_merged = {merge_id: static_mesh_components_unmerged}

    raw_obj_dir = os.path.realpath(os.path.join(
        args.pipeline_dir, args.scene_id, "collision_geometry", "raw", actor_name.replace("/", "."), node["name"]))
    os.makedirs(raw_obj_dir, exist_ok=True)

    for merge_id, merge_id_static_mesh_components in static_mesh_components_merged.items():

        # Concatenate multiple static meshes that have the same merge_id into a single mesh.
        raw_merge_id_meshes = trimesh.Scene()
        for static_mesh_component_name, static_mesh_component_desc in merge_id_static_mesh_components:

            transform_current_node_from_current_component = \
                spear.pipeline.get_transform_from_transform_data(
                    static_mesh_component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"])

            M_current_node_from_current_component = spear.pipeline.get_matrix_from_transform(transform_current_node_from_current_component)

            static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_component_desc["editor_properties"]["static_mesh"]["path"])
            assert static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id)

            obj_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".obj"
            numerical_parity_obj_path = \
                os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))
            spear.log(log_prefix_str, "    Reading OBJ file: ", numerical_parity_obj_path)

            mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
            V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
            V_current_node = M_current_node_from_current_component*V_current_component
            assert np.allclose(V_current_node[3,:], 1.0)
            mesh.vertices = V_current_node.T.A[:,0:3]

            raw_merge_id_meshes.add_geometry(mesh)

        raw_merge_id_mesh = raw_merge_id_meshes.dump(concatenate=True)

        # Save merged mesh.
        raw_merge_id_obj_path = os.path.realpath(os.path.join(raw_obj_dir, f"{merge_id:04}.obj"))
        spear.log(log_prefix_str, "Writing OBJ file: ", raw_merge_id_obj_path)
        raw_merge_id_mesh.export(raw_merge_id_obj_path, "obj")

        # Run COACD.
        coacd_merge_id_mesh = coacd.Mesh(raw_merge_id_mesh.vertices, raw_merge_id_mesh.faces)
        coacd_merge_id_parts = coacd.run_coacd(coacd_merge_id_mesh)

        # Save COACD result as individual parts and as a combined mesh.
        coacd_obj_dir = os.path.realpath(os.path.join(
            args.pipeline_dir, args.scene_id, "collision_geometry", "coacd", actor_name.replace("/", "."), node["name"]))
        os.makedirs(coacd_obj_dir, exist_ok=True)

        coacd_merge_id_obj_dir = os.path.realpath(os.path.join(
            args.pipeline_dir, args.scene_id, "collision_geometry", "coacd", actor_name.replace("/", "."), node["name"], f"{merge_id:04}"))
        os.makedirs(coacd_merge_id_obj_dir, exist_ok=True)

        coacd_merge_id_meshes = trimesh.Scene()
        for coacd_part_id, (coacd_part_vertices, coacd_part_faces) in enumerate(coacd_merge_id_parts):

            coacd_part_mesh = trimesh.Trimesh(vertices=coacd_part_vertices, faces=coacd_part_faces)
            coacd_merge_id_meshes.add_geometry(coacd_part_mesh)

            coacd_part_id_obj_path = os.path.realpath(os.path.join(coacd_merge_id_obj_dir, f"{coacd_part_id:04}.obj"))
            spear.log(log_prefix_str, "Writing OBJ file: ", coacd_part_id_obj_path)
            coacd_part_mesh.export(coacd_part_id_obj_path, "obj")

        coacd_merge_id_obj_path = os.path.realpath(os.path.join(coacd_obj_dir, f"{merge_id:04}.obj"))
        spear.log(log_prefix_str, "Writing OBJ file: ", coacd_merge_id_obj_path)
        coacd_merge_id_meshes.export(coacd_merge_id_obj_path, "obj")

        # Recurse for each child node.
        for child_node in node["children_nodes"].values():
            generate_collision_geometry(actor_name=actor_name, node=child_node["node"], log_prefix_str=log_prefix_str+"    ")


if __name__ == '__main__':
    process_scene()
