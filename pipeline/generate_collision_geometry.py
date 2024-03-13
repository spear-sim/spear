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
    kinematic_trees_actors_json_file = os.path.realpath(os.path.join(kinematic_trees_dir, "actors.json"))
    assert os.path.exists(kinematic_trees_dir)
    spear.log("Reading JSON file: " + kinematic_trees_actors_json_file)
    with open(kinematic_trees_actors_json_file, "r") as f:
        actors_json = json.load(f)

    actors = actors_json
    actors = { actor_name: generate_collision_geometry(actor_name, actor_kinematic_tree) for actor_name, actor_kinematic_tree in actors.items() }

    collision_geometry_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "collision_geometry"))
    collision_geometry_actors_json_file = os.path.realpath(os.path.join(collision_geometry_dir, "actors.json"))
    spear.log("Writing JSON file: " + collision_geometry_actors_json_file)
    os.makedirs(collision_geometry_dir, exist_ok=True)
    with open(collision_geometry_actors_json_file, "w") as f:
        json.dump(actors, f, indent=4, sort_keys=True)

    spear.log("Done.")


def generate_collision_geometry(actor_name, kinematic_tree):
    spear.log("Processing actor: ", actor_name)
    return {
        "root_node": generate_collision_geometry_for_kinematic_tree_node(
            actor_name=actor_name,
            kinematic_tree_node=kinematic_tree["root_node"],
            log_prefix_str="    ")}


def generate_collision_geometry_for_kinematic_tree_node(actor_name, kinematic_tree_node, log_prefix_str):

    spear.log(log_prefix_str, "Processing kinematic tree node: ", kinematic_tree_node["name"])

    # TODO: retrieve merge_ids from the JSON data, group static_mesh_components by merge_id
    static_mesh_components = kinematic_tree_node["static_mesh_components"]
    if len(static_mesh_components) > 0:
        merge_id = 0
        static_mesh_components_grouped_by_merge_id = {merge_id: static_mesh_components}
    else:
        static_mesh_components_grouped_by_merge_id = {}

    raw_obj_dir = os.path.realpath(os.path.join(
        args.pipeline_dir, args.scene_id, "collision_geometry", "raw", actor_name.replace("/", "."), kinematic_tree_node["name"]))
    os.makedirs(raw_obj_dir, exist_ok=True)

    # TODO: Break this group-by-merge-id step into a different pipeline stage, so we can generate the
    # grouped-by-merge-id meshes once, and then run different convex decomposition strategies on them
    # in different pipeline stages without duplicating effort or overwriting JSON data.

    kinematic_tree_node["pipeline_info"]["generate_collision_geometry"] = {}
    kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"] = {}

    for merge_id, static_mesh_components_for_merge_id in static_mesh_components_grouped_by_merge_id.items():

        assert len(static_mesh_components_for_merge_id) > 0

        kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id] = {}
        kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id]["static_mesh_components"] = \
            static_mesh_components_for_merge_id

        # Combine multiple static mesh components that have the same merge_id into a single mesh.
        raw_merge_id_mesh_scene = trimesh.Scene()
        for static_mesh_component_name, static_mesh_component_desc in static_mesh_components_for_merge_id.items():

            transform_current_node_from_current_component = \
                spear.pipeline.get_transform_from_transform_data(
                    static_mesh_component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"])

            M_current_node_from_current_component = spear.pipeline.get_matrix_from_transform(transform_current_node_from_current_component)

            static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_component_desc["editor_properties"]["static_mesh"]["path"])
            assert static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id)

            obj_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".obj"
            numerical_parity_obj_path = \
                os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))
            spear.log(log_prefix_str, "Reading OBJ file: ", numerical_parity_obj_path)

            mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
            V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
            V_current_node = M_current_node_from_current_component*V_current_component
            assert np.allclose(V_current_node[3,:], 1.0)
            mesh.vertices = V_current_node.T.A[:,0:3]

            raw_merge_id_mesh_scene.add_geometry(mesh)

        raw_merge_id_mesh = raw_merge_id_mesh_scene.dump(concatenate=True)

        # Save merged mesh.
        raw_merge_id_obj_path = os.path.realpath(os.path.join(raw_obj_dir, f"merge_id_{merge_id:04}.obj"))
        spear.log(log_prefix_str, "Writing OBJ file: ", raw_merge_id_obj_path)
        raw_merge_id_mesh.export(raw_merge_id_obj_path, "obj")

        # Run COACD. TODO: retrieve COACD parameters from the JSON data
        coacd_mesh = coacd.Mesh(raw_merge_id_mesh.vertices, raw_merge_id_mesh.faces)
        coacd_parts_data = coacd.run_coacd(coacd_mesh)

        # Save COACD result as individual parts, and as a combined mesh for debugging.
        coacd_obj_dir = os.path.realpath(os.path.join(
            args.pipeline_dir, args.scene_id, "collision_geometry", "coacd", actor_name.replace("/", "."), kinematic_tree_node["name"]))
        os.makedirs(coacd_obj_dir, exist_ok=True)

        coacd_part_obj_dir = os.path.realpath(os.path.join(coacd_obj_dir, f"merge_id_{merge_id:04}"))
        os.makedirs(coacd_part_obj_dir, exist_ok=True)

        coacd_parts_scene = trimesh.Scene()
        kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id]["coacd"] = {}
        kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id]["coacd"]["part_ids"] = []
        for coacd_part_id, (coacd_part_vertices, coacd_part_faces) in enumerate(coacd_parts_data):

            coacd_part_mesh = trimesh.Trimesh(vertices=coacd_part_vertices, faces=coacd_part_faces)
            coacd_parts_scene.add_geometry(coacd_part_mesh)

            coacd_part_obj_path = os.path.realpath(os.path.join(coacd_part_obj_dir, f"part_id_{coacd_part_id:04}.obj"))
            spear.log(log_prefix_str, "Writing OBJ file: ", coacd_part_obj_path)
            coacd_part_mesh.export(coacd_part_obj_path, "obj")

            kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id]["coacd"]["part_ids"].append(coacd_part_id)

        coacd_obj_path = os.path.realpath(os.path.join(coacd_obj_dir, f"merge_id_{merge_id:04}.obj"))
        spear.log(log_prefix_str, "Writing OBJ file: ", coacd_obj_path)
        coacd_parts_scene.export(coacd_obj_path, "obj")

    # Recurse for each child node. We need to copy() so we're not modifying the dict as we're iterating over it.
    for child_kinematic_tree_node_name, child_kinematic_tree_node in kinematic_tree_node["children_nodes"].copy().items():
        kinematic_tree_node["children_nodes"][child_kinematic_tree_node_name]["node"] = generate_collision_geometry_for_kinematic_tree_node(
            actor_name=actor_name,
            kinematic_tree_node=child_kinematic_tree_node["node"],
            log_prefix_str=log_prefix_str+"    ")

    return kinematic_tree_node


if __name__ == '__main__':
    process_scene()
