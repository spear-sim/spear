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


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--ignore_actors")
args = parser.parse_args()

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]
physics_constraint_component_classes = ["PhysicsConstraintComponent"]


def process_scene():

    unreal_scene_json_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_scene_json"))
    assert os.path.exists(unreal_scene_json_dir)

    unreal_scene_json_file = os.path.realpath(os.path.join(unreal_scene_json_dir, "unreal_scene.json"))
    with open(unreal_scene_json_file, "r") as f:
        unreal_scene_json = json.load(f)

    # It is possible for an actor not to have a root component.
    for actor_name, actor_desc in unreal_scene_json.items():
        # if actor_name not in ignore_actors and "root_component" in actor_desc.keys():
        if actor_name == "Meshes/03_cabinet/Cabinet":
            spear.log("Processing actor: ", actor_name)
            component_desc = actor_desc["root_component"]

            kinematic_tree_node = get_kinematic_tree_node(
                actor_name,
                component_desc,
                world_from_component_transform_func=world_from_component_transform_using_relative_lrs,
                world_from_component_transform_data={
                    "location": np.matrix(np.zeros(3)).T,
                    "rotation": np.matrix(np.identity(3)),
                    "scale": np.matrix(np.identity(3))},
                log_prefix_str="    ")

            kinematic_tree = {"root_node": kinematic_tree_node}

            kt_json_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "kinematic_tree"))
            kt_json_file = os.path.realpath(os.path.join(kt_json_dir, "kinematic_tree.json"))

            spear.log("Generating JSON file: " + kt_json_file)

            os.makedirs(kt_json_dir, exist_ok=True)
            with open(kt_json_file, "w") as f:
                json.dump(kinematic_tree, f, indent=4, sort_keys=True)


def get_kinematic_tree_node(actor_name, component_desc, world_from_component_transform_func, world_from_component_transform_data, log_prefix_str=""):

    component_name = component_desc["name"]
    kinematic_tree_node = {"name": component_name, "static_mesh_components": {}, "children_nodes": {}}

    # Only process SceneComponents...
    component_class = component_desc["class"]
    if component_class in scene_component_classes:

        spear.log(log_prefix_str, "Processing SceneComponent: ", component_name)
        M_world_from_component, world_from_component_transform_data = \
            world_from_component_transform_func(component_desc, world_from_component_transform_data)

        # ...and only attempt to add to the kinematic tree node's dict of StaticMeshComponents if the current component is a StaticMeshComponent...
        if component_class in static_mesh_component_classes:
            spear.log(log_prefix_str, "Component is a StaticMeshComponent.")

            # ...that refers to a non-null StaticMesh asset...
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]
            if static_mesh_desc is not None:

                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
                spear.log(log_prefix_str, "StaticMesh asset path: ", static_mesh_asset_path)
    
                # ...that is in the /Game/Scenes/<scene_id> directory.
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):

                    spear.log(log_prefix_str, "Adding component to kinematic tree...")
                    kinematic_tree_node["static_mesh_components"][component_name] = {"name": component_name, "M_node_from_component": str(M_world_from_component)}

        # Recurse for each child component.
        physics_constraint_components = \
            {name: desc for name, desc in component_desc["children_components"].items() if desc["class"] in physics_constraint_component_classes}

        for child_component_name, child_component_desc in component_desc["children_components"].items():
            spear.log(log_prefix_str, "Processing child component: ", child_component_name)

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
                    spear.log(log_prefix_str, "Constrained to current component via PhysicsConstraintComponent: ", physics_constraint_component_name)
                    break

            # If the child component is constrained to the current component via a PhysicsConstraintComponent, then don't accumulate
            # {location,rotation,scale}. In this case, we add the child component's kinematic tree node as a new child of the current
            # node.
            if child_component_constrained_via_physics_constraint_component:
                child_component_kinematic_tree_node = get_kinematic_tree_node(
                    actor_name,
                    child_component_desc,
                    world_from_component_transform_func=world_from_component_transform_using_relative_lrs,
                    world_from_component_transform_data={
                        "location": np.matrix(np.zeros(3)).T,
                        "rotation": np.matrix(np.identity(3)),
                        "scale": np.matrix(np.identity(3))},
                    log_prefix_str=log_prefix_str+"    ")

                assert child_component_name not in kinematic_tree_node["children_nodes"].keys()
                kinematic_tree_node["children_nodes"][child_component_name] = child_component_kinematic_tree_node

            # Otherwise, there is no PhysicsConstraintComponent connecting the child component and the current component, so we absorb the
            # child component's kinematic tree node into the current node, i.e., we merge their "static_mesh_components" and "children_nodes"
            # dicts.
            else:
                child_component_kinematic_tree_node = get_kinematic_tree_node(
                        actor_name,
                        child_component_desc,
                        world_from_component_transform_func=world_from_component_transform_func,
                        world_from_component_transform_data=world_from_component_transform_data,
                        log_prefix_str=log_prefix_str+"    ")

                ktn = kinematic_tree_node
                child_component_ktn = child_component_kinematic_tree_node

                assert set(ktn["static_mesh_components"].keys()) & set(child_component_ktn["static_mesh_components"].keys()) == set()
                ktn["static_mesh_components"].update(child_component_ktn["static_mesh_components"])

                assert set(ktn["children_nodes"].keys()) & set(child_component_ktn["children_nodes"].keys()) == set()
                ktn["children_nodes"].update(child_component_ktn["children_nodes"])

    return kinematic_tree_node


def world_from_component_transform_using_relative_lrs(component_desc, world_from_component_transform_data):

    absolute_location = component_desc["editor_properties"]["absolute_location"]
    absolute_rotation = component_desc["editor_properties"]["absolute_rotation"]
    absolute_scale = component_desc["editor_properties"]["absolute_scale"]

    relative_location_x = component_desc["editor_properties"]["relative_location"]["editor_properties"]["x"]
    relative_location_y = component_desc["editor_properties"]["relative_location"]["editor_properties"]["y"]
    relative_location_z = component_desc["editor_properties"]["relative_location"]["editor_properties"]["z"]
    relative_rotation_roll = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["roll"]
    relative_rotation_pitch = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["pitch"]
    relative_rotation_yaw = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["yaw"]
    relative_scale3d_x = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["x"]
    relative_scale3d_y = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["y"]
    relative_scale3d_z = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["z"]

    #
    # Unreal defines roll-pitch-yaw Euler angles according to the following conventions, which can be
    # verified by manual inspection in the editor.
    #     A positive roll  is a rotation around X, starting from +Z and rotating towards +Y
    #     A positive pitch is a rotation around Y, starting from +X and rotating towards +Z
    #     A positive yaw   is a rotation around Z, starting from +X and rotating towards +Y
    #
    # On the other hand, the scipy.spatial.transform.Rotation class defines a rotation around each axis
    # according to the following conventions, which can be verified by manually inspecting the output of
    # scipy.spatial.transform.Rotation.from_euler(...).as_matrix().
    #     A rotation around X by theta radians is defined by the following matrix,
    #         [[1 0 0  ]
    #          [0 c -s ] 
    #          [0 s c  ]], where c=cos(theta) and s=sin(theta)
    #     A rotation around Y by theta radians is defined by the following matrix,
    #         [[c  0 s ]
    #          [0  1 0 ] 
    #          [-s 0 c ]], where c=cos(theta) and s=sin(theta)
    #     A rotation around Z by theta radians is defined by the following matrix,
    #         [[c -s 0 ]
    #          [s c  0 ] 
    #          [0 0  1 ]], where c=cos(theta) and s=sin(theta)
    #
    # These conventions conflict. We therefore need to negate Unreal's roll (rotation around X) and pitch
    # (rotation around Y) but not yaw (rotation around Z) when constructing a scipy.spatial.transform.Rotation
    # object from Unreal roll-pitch-yaw angles. Unreal editor properties also specify roll-pitch-yaw Euler
    # angles in degrees, whereas the scipy.spatial.transform.Rotation.from_euler(...) function expects radians
    # by default. So we also need to convert from degrees to radians.
    #

    roll  = np.deg2rad(-relative_rotation_roll)
    pitch = np.deg2rad(-relative_rotation_pitch)
    yaw   = np.deg2rad(relative_rotation_yaw)

    # 
    # Unreal applies roll-pitch-yaw Euler angles in world-space in the following order, which can be verified
    # by manual inspection the editor.
    #     1. Rotate around world-space X by roll degrees
    #     2. Rotate around world-space Y by pitch degrees
    #     3. Rotate around world-space Z by yaw degrees
    # 
    # So, given a triplet of roll-pitch-yaw values that has been negated appropriately and converted to radians
    # as described above, we define the rotation matrix that corresponds to the roll-pitch-yaw values as
    # follows,
    #     R_x = np.matrix(scipy.spatial.transform.Rotation.from_euler("x", roll).as_matrix())
    #     R_y = np.matrix(scipy.spatial.transform.Rotation.from_euler("y", pitch).as_matrix())
    #     R_z = np.matrix(scipy.spatial.transform.Rotation.from_euler("z", yaw).as_matrix())
    #     R   = R_z*R_y*R_x
    # which is equivalent to the following expression,
    #     R   = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    #

    l_parent_from_component = np.matrix([relative_location_x, relative_location_y, relative_location_z]).T
    R_parent_from_component = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    S_parent_from_component = np.matrix(np.diag([relative_scale3d_x, relative_scale3d_y, relative_scale3d_z]))

    eps = 0.000001
    assert np.all(np.diag(S_parent_from_component) > eps)

    # This formulation for accumulating {location, rotation, scale} through the component hierarchy is not
    # immediately obvious to me, but it matches the behavior of USceneComponent and FTransform, see:
    #     Engine/Source/Runtime/Engine/Private/Components/SceneComponent.cpp
    #     Engine/Source/Runtime/Core/Public/Math/TransformNonVectorized.h

    l_world_from_parent = world_from_component_transform_data["location"]
    R_world_from_parent = world_from_component_transform_data["rotation"]
    S_world_from_parent = world_from_component_transform_data["scale"]

    l_world_from_component = R_world_from_parent*S_world_from_parent*l_parent_from_component + l_world_from_parent
    R_world_from_component = R_world_from_parent*R_parent_from_component
    S_world_from_component = S_parent_from_component*S_world_from_parent

    # If we're in absolute mode for {location, rotation, scale}, then don't accumulate.
    if absolute_location:
        l_world_from_component = l_parent_from_component
    if absolute_rotation:
        R_world_from_component = R_parent_from_component
    if absolute_scale:
        S_world_from_component = S_parent_from_component

    # Pack all accumulated data.
    world_from_component_transform_data = {"location": l_world_from_component, "rotation": R_world_from_component, "scale": S_world_from_component}

    # Construct the 4x4 world-from-component transformation matrix by applying transformations in the
    # following order: (1) scale; (2) rotation; (3) translation. See the following link for more details:
    #     https://docs.unrealengine.com/5.2/en-US/API/Runtime/Core/Math/FTransform

    M_l_world_from_component = np.matrix(np.block([[np.identity(3),         l_world_from_component], [np.zeros([1,3]), 1.0]]))
    M_R_world_from_component = np.matrix(np.block([[R_world_from_component, np.zeros([3,1])],        [np.zeros([1,3]), 1.0]]))
    M_S_world_from_component = np.matrix(np.block([[S_world_from_component, np.zeros([3,1])],        [np.zeros([1,3]), 1.0]]))

    M_world_from_component = M_l_world_from_component*M_R_world_from_component*M_S_world_from_component

    return M_world_from_component, world_from_component_transform_data


if __name__ == '__main__':
    process_scene()
