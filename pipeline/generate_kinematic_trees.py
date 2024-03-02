#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import numpy as np
import os
import pathlib
import scipy.spatial.transform
import spear
import spear.pipeline

parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
args = parser.parse_args()

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]
physics_constraint_component_classes = ["PhysicsConstraintComponent"]


def process_scene():

    unreal_scene_json_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_scene_json"))
    assert os.path.exists(unreal_scene_json_dir)

    actors_json_file = os.path.realpath(os.path.join(unreal_scene_json_dir, "actors.json"))
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    actor_descs = actors_json.items()
    actor_descs = [ (actor_name, actor_desc) for actor_name, actor_desc in actor_descs if actor_desc["root_component"] is not None ]
    actor_kinematic_trees = { actor_name: get_kinematic_tree(actor_desc) for actor_name, actor_desc in actor_descs }

    kinematic_trees_json_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "kinematic_trees"))
    kinematic_trees_json_file = os.path.realpath(os.path.join(kinematic_trees_json_dir, "actor_kinematic_trees.json"))
    spear.log("Generating JSON file: " + kinematic_trees_json_file)

    os.makedirs(kinematic_trees_json_dir, exist_ok=True)
    with open(kinematic_trees_json_file, "w") as f:
        json.dump(actor_kinematic_trees, f, indent=4, sort_keys=True)


def get_kinematic_tree(actor_desc):
    actor_name = actor_desc["name"]
    spear.log("Processing actor: ", actor_name)
    return {"root_node": get_kinematic_tree_node(
        actor_name=actor_name,
        transform_parent_node_from_current_node=spear.pipeline.TRANSFORM_IDENTITY,
        transform_current_node_from_parent_component=spear.pipeline.TRANSFORM_IDENTITY,
        component_desc=actor_desc["root_component"],
        log_prefix_str="    ")}


def get_kinematic_tree_node(actor_name, transform_parent_node_from_current_node, transform_current_node_from_parent_component, component_desc, log_prefix_str=""):

    component_name = component_desc["name"]
    kinematic_tree_node = {
        "children_nodes": {},
        "name": component_name,
        "static_mesh_components": {},
        "transform_parent_node_from_current_node": spear.pipeline.get_transform_data_from_transform(transform_parent_node_from_current_node)}

    # Only process SceneComponents...
    if component_desc["class"] in scene_component_classes:

        spear.log(log_prefix_str, "Processing SceneComponent: ", component_name)
        
        transform_current_node_from_current_component = \
            spear.pipeline.get_transform_ancestor_component_from_current_component(transform_current_node_from_parent_component, component_desc)

        component_desc["pipeline_info"]["generate_kinematic_trees"] = {}
        component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"] = \
            spear.pipeline.get_transform_data_from_transform(transform_current_node_from_current_component)

        # ...and only attempt to add to the kinematic tree node's dict of StaticMeshComponents if the current component is a StaticMeshComponent...
        if component_desc["class"] in static_mesh_component_classes:
            spear.log(log_prefix_str, "Component is a StaticMeshComponent.")

            # ...that refers to a non-null StaticMesh asset...
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]
            if static_mesh_desc is not None:

                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
                spear.log(log_prefix_str, "StaticMesh asset path: ", static_mesh_asset_path)
    
                # ...that is in the /Game/Scenes/<scene_id> directory.
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):

                    spear.log(log_prefix_str, "Adding component to kinematic tree...")
                    kinematic_tree_node["static_mesh_components"][component_name] = component_desc

        # Recurse for each child component.
        physics_constraint_components = \
            {name: desc for name, desc in component_desc["children_components"].items() if desc["class"] in physics_constraint_component_classes}

        for child_component_name, child_component_desc in component_desc["children_components"].items():
            spear.log(log_prefix_str, "Processing child component: ", child_component_name)

            # Determine whether or not the child component is constrained to the current component via a PhysicsConstraintComponent.
            child_component_constrained_via_physics_constraint_component = False
            for physics_constraint_component_name, physics_constraint_component_desc in physics_constraint_components.items():

                constraint_actor1 = physics_constraint_component_desc["editor_properties"]["constraint_actor1"]
                constraint_actor2 = physics_constraint_component_desc["editor_properties"]["constraint_actor2"]
                component_name1 = physics_constraint_component_desc["editor_properties"]["component_name1"]["editor_properties"]["component_name"]
                component_name2 = physics_constraint_component_desc["editor_properties"]["component_name2"]["editor_properties"]["component_name"]

                # Assume that component 1 is the parent and component 2 is the child.
                child_component_constrained_via_physics_constraint_component = \
                    (constraint_actor1 is None or constraint_actor1["name"] == actor_name) and \
                    (constraint_actor2 is None or constraint_actor2["name"] == actor_name) and \
                    component_name1 == component_desc["unreal_name"] and \
                    component_name2 == child_component_desc["unreal_name"]

                if child_component_constrained_via_physics_constraint_component:
                    spear.log(log_prefix_str, component_name, " constrained to ", child_component_name, " via ", physics_constraint_component_name)
                    break

            # If the child component is constrained to the current component via a PhysicsConstraintComponent, then don't accumulate
            # {location,rotation,scale}. In this case, we add the child component's kinematic tree node as a new child of the current node.
            if child_component_constrained_via_physics_constraint_component:
                spear.log(log_prefix_str, "Adding kinematic tree node for child component as a new node...")
                child_component_kinematic_tree_node = get_kinematic_tree_node(
                    actor_name=actor_name,
                    transform_parent_node_from_current_node=transform_current_node_from_current_component,
                    transform_current_node_from_parent_component=spear.pipeline.TRANSFORM_IDENTITY,
                    component_desc=child_component_desc,
                    log_prefix_str=log_prefix_str+"    ")

                assert child_component_name not in kinematic_tree_node["children_nodes"].keys()
                kinematic_tree_node["children_nodes"][child_component_name] = child_component_kinematic_tree_node

            # Otherwise, there is no PhysicsConstraintComponent connecting the child component and the current component, so we absorb the
            # child component's kinematic tree node into the current node, i.e., we merge their "static_mesh_components" and "children_nodes"
            # dicts respectfully.
            else:
                spear.log(log_prefix_str, "Merging kinematic tree node for child component with existing node...")
                child_component_kinematic_tree_node = get_kinematic_tree_node(
                    actor_name=actor_name,
                    transform_parent_node_from_current_node=None,
                    transform_current_node_from_parent_component=transform_current_node_from_current_component,
                    component_desc=child_component_desc,
                    log_prefix_str=log_prefix_str+"    ")

                ktn = kinematic_tree_node
                child_component_ktn = child_component_kinematic_tree_node

                assert set(ktn["static_mesh_components"].keys()) & set(child_component_ktn["static_mesh_components"].keys()) == set()
                ktn["static_mesh_components"].update(child_component_ktn["static_mesh_components"])

                assert set(ktn["children_nodes"].keys()) & set(child_component_ktn["children_nodes"].keys()) == set()
                ktn["children_nodes"].update(child_component_ktn["children_nodes"])

    return kinematic_tree_node


if __name__ == '__main__':
    process_scene()
