#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import pathlib
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

    unreal_metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, "scenes", args.scene_id, "unreal_metadata"))
    unreal_metadata_actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "actors.json"))
    assert os.path.exists(unreal_metadata_dir)
    spear.log("Reading JSON file: " + unreal_metadata_actors_json_file)
    with open(unreal_metadata_actors_json_file, "r") as f:
        actors_json = json.load(f)

    actors = actors_json
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["root_component"] is not None }
    actors = { actor_name: get_kinematic_tree(actor_desc) for actor_name, actor_desc in actors.items() }
    actors = { actor_name: actor_kinematic_tree for actor_name, actor_kinematic_tree in actors.items() if actor_kinematic_tree["root_node"] is not None }

    kinematic_trees_dir = os.path.realpath(os.path.join(args.pipeline_dir, "scenes", args.scene_id, "kinematic_trees"))
    kinematic_trees_actors_json_file = os.path.realpath(os.path.join(kinematic_trees_dir, "actors.json"))
    spear.log("Writing JSON file: " + kinematic_trees_actors_json_file)
    os.makedirs(kinematic_trees_dir, exist_ok=True)
    with open(kinematic_trees_actors_json_file, "w") as f:
        json.dump(actors, f, indent=4, sort_keys=True)

    spear.log("Done.")

def get_kinematic_tree(actor_desc):
    actor_name = actor_desc["name"]
    spear.log("Processing actor: ", actor_name)
    return {
        "root_node": get_kinematic_tree_node(
            actor_name=actor_name,
            component_desc=actor_desc["root_component"],
            component_is_root_within_node=True,
            transform_node_from_parent_component=spear.pipeline.TRANSFORM_IDENTITY,
            log_prefix_str="    ")}

def get_kinematic_tree_node(actor_name, component_desc, component_is_root_within_node, transform_node_from_parent_component, log_prefix_str):

    component_name = component_desc["name"]
    has_valid_geometry = False
    kinematic_tree_node = {
        "children_nodes": {},
        "name": component_name,
        "pipeline_info": {},
        "static_mesh_components": {},
        "transform_parent_node_from_current_node": spear.pipeline.get_transform_data_from_transform(spear.pipeline.TRANSFORM_IDENTITY)}

    spear.log(log_prefix_str, "Getting kinematic tree node for component: ", component_name)

    # Only process SceneComponents with no absolute transforms.
    if component_desc["class"] in scene_component_classes and not spear.pipeline.any_component_transform_absolute(component_desc):

        transform_node_from_current_component = spear.pipeline.compose_transform_with_component(transform_node_from_parent_component, component_desc)

        # If the current component is the root component within its node, then the accumulated transform maps from the current node to
        # the parent node, and only an identity transform is needed to map from the current component to the current node.
        if component_is_root_within_node:
            kinematic_tree_node["transform_parent_node_from_current_node"] = \
                spear.pipeline.get_transform_data_from_transform(transform_node_from_current_component)
            transform_node_from_current_component = spear.pipeline.TRANSFORM_IDENTITY

        # Only attempt to add geometry to the current node if the current component is a StaticMeshComponent...
        if component_desc["class"] in static_mesh_component_classes:

            # ...that refers to a non-null StaticMesh asset...
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]
            if static_mesh_desc is not None:
                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
    
                # ...that is in the /Game/Scenes/<scene_id> directory.
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):
                    spear.log(log_prefix_str, "Component is a StaticMeshComponent with valid geometry.")

                    has_valid_geometry = True

                    # The accumulated transform maps from the current component to the current node.
                    component_desc["pipeline_info"]["generate_kinematic_trees"] = {}
                    component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"] = \
                        spear.pipeline.get_transform_data_from_transform(transform_node_from_current_component)

                    kinematic_tree_node["static_mesh_components"][component_name] = component_desc

        # Recurse for each child component.
        physics_constraint_components = component_desc["children_components"]
        physics_constraint_components = { name: desc for name, desc in physics_constraint_components.items() if desc["class"] in physics_constraint_component_classes }
        physics_constraint_components = { name: desc for name, desc in physics_constraint_components.items() if not spear.pipeline.any_component_transform_absolute(desc) }

        children_components = component_desc["children_components"]
        children_components = { name: desc for name, desc in children_components.items() if desc["class"] not in physics_constraint_component_classes }

        for child_component_name, child_component_desc in children_components.items():
            spear.log(log_prefix_str, "Processing child component: ", child_component_name)

            # Determine whether or not the child component is constrained to the current component via a valid PhysicsConstraintComponent.
            # When constructing our kinematic tree representation, we ignore all PhysicsConstraintComponents with absolute transforms.
            physics_constraint_component_descs = []
            for physics_constraint_component_name, physics_constraint_component_desc in physics_constraint_components.items():

                constraint_actor1 = physics_constraint_component_desc["editor_properties"]["constraint_actor1"]
                constraint_actor2 = physics_constraint_component_desc["editor_properties"]["constraint_actor2"]
                component_name1 = physics_constraint_component_desc["editor_properties"]["component_name1"]["editor_properties"]["component_name"]
                component_name2 = physics_constraint_component_desc["editor_properties"]["component_name2"]["editor_properties"]["component_name"]

                # TODO: need to change so that component 1 is the child and component 2 is the parent to match Unreal's convention
                child_component_constrained_by_physics_constraint_component = \
                    (constraint_actor1 is None or constraint_actor1["name"] == actor_name) and \
                    (constraint_actor2 is None or constraint_actor2["name"] == actor_name) and \
                    component_name1 == component_desc["unreal_name"] and \
                    component_name2 == child_component_desc["unreal_name"]

                if child_component_constrained_by_physics_constraint_component:
                    physics_constraint_component_descs.append(physics_constraint_component_desc)
                    spear.log(log_prefix_str, "Current component constrained to child component via PhysicsConstraintComponent: ", physics_constraint_component_name)

            spear.log(
                log_prefix_str,
                "PhysicsConstraintComponents constraining current component and child component: ",
                len(physics_constraint_component_descs))
            spear.log(log_prefix_str, "Getting kinematic tree node for child component: ", child_component_name)

            # When constructing our kinematic tree representation, we allow at most one PhysicsConstraintComponent between a parent
            # component and a child component. If a parent component and a child component are constrained by more than one
            # PhysicsConstraintComponent, then we ignore both.
            child_component_is_root_within_node = len(physics_constraint_component_descs) == 1

            child_component_kinematic_tree_node = get_kinematic_tree_node(
                actor_name=actor_name,
                component_desc=child_component_desc,
                component_is_root_within_node=child_component_is_root_within_node,
                transform_node_from_parent_component=transform_node_from_current_component,
                log_prefix_str=log_prefix_str+"    ")

            if child_component_kinematic_tree_node is not None:
                spear.log(log_prefix_str, "Kinematic tree node for child component has valid geometry.")
                has_valid_geometry = True

                # If the child component is the root component within its node, i.e., if there is a PhysicsConstraintComponent
                # constraining the current component and the child component, then we add the child component's node as a child
                # of the current node.
                if child_component_is_root_within_node:
                    spear.log(log_prefix_str, "Adding child component to kinematic tree as a new node.")

                    assert child_component_name not in kinematic_tree_node["children_nodes"].keys()
                    assert len(physics_constraint_component_descs) == 1

                    physics_constraint_component_desc = physics_constraint_component_descs[0]

                     # Compute the transform that maps to the current node from the PhysicsConstraintComponent, i.e., find the pose
                     # of the PhysicsConstraintComponent in the current node's frame.
                    transform_node_from_child_physics_constraint_component = \
                        spear.pipeline.compose_transform_with_component(transform_node_from_current_component, physics_constraint_component_desc)

                    physics_constraint_component_desc["pipeline_info"]["generate_kinematic_trees"] = {}
                    physics_constraint_component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"] = \
                        spear.pipeline.get_transform_data_from_transform(transform_node_from_child_physics_constraint_component)

                    kinematic_tree_node["children_nodes"][child_component_name] = {
                        "node": child_component_kinematic_tree_node,
                        "physics_constraint_component": physics_constraint_component_desc}

                # Otherwise, we absorb the child component's node into the current node, i.e., we merge "static_mesh_components" and
                # "children_nodes" respectfully.
                else:
                    spear.log(log_prefix_str, "Adding child component to kinematic tree by merging with current node.")

                    ktn = kinematic_tree_node
                    child_component_ktn = child_component_kinematic_tree_node
                    assert set(ktn["static_mesh_components"].keys()) & set(child_component_ktn["static_mesh_components"].keys()) == set()
                    ktn["static_mesh_components"].update(child_component_ktn["static_mesh_components"])
                    assert set(ktn["children_nodes"].keys()) & set(child_component_ktn["children_nodes"].keys()) == set()
                    ktn["children_nodes"].update(child_component_ktn["children_nodes"])

            else:
                spear.log(log_prefix_str, "Kinematic tree node for child component does not have valid geometry.")

    if has_valid_geometry:
        spear.log(log_prefix_str, "Kinematic tree node for component has valid geometry.")
        return kinematic_tree_node
    else:
        spear.log(log_prefix_str, "Kinematic tree node for component does not have valid geometry.")
        return None


if __name__ == "__main__":
    process_scene()
