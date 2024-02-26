#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import posixpath
import trimesh
import scipy.spatial.transform
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--visual_parity_with_unreal", action="store_true")
parser.add_argument("--ignore_actors")
args = parser.parse_args()

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0, 0.0, 0.0)
c_y_axis = (0.0, 1.0, 0.0)
c_z_axis = (0.0, 0.0, 1.0)
c_face   = (0.75,0.75,0.75)

origin_world = np.array([[0.0,   0.0,   0.0]])
x_axis_world = np.array([[100.0, 0.0,   0.0]])
y_axis_world = np.array([[0.0,   100.0, 0.0]])
z_axis_world = np.array([[0.0,   0.0,   100.0]])

# Swap y and z coordinates to match the visual appearance of the Unreal editor.
if args.visual_parity_with_unreal:
    origin_world = origin_world[:,[0,2,1]]
    x_axis_world = x_axis_world[:,[0,2,1]]
    y_axis_world = y_axis_world[:,[0,2,1]]
    z_axis_world = z_axis_world[:,[0,2,1]]


def process_scene():
    json_file = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_scene_json", "unreal_scene.json"))

    with open(json_file, "r") as f:
        unreal_scene_json = json.load(f)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         x_axis_world[:,0], x_axis_world[:,1], x_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_x_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         y_axis_world[:,0], y_axis_world[:,1], y_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_y_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         z_axis_world[:,0], z_axis_world[:,1], z_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_z_axis)

    # It is possible for an actor not to have a root component.
    for actor_name, actor_desc in unreal_scene_json.items():
        if actor_name not in args.ignore_actors and "root_component" in actor_desc.keys():
            spear.log("Processing actor: ", actor_name)
            component_desc = actor_desc["root_component"]

            draw_components(
                component_desc,
                world_from_component_transform_func=world_from_component_transform_using_relative_lrs,
                world_from_component_transform_data={
                    "location": np.matrix(np.zeros(3)).T,
                    "rotation": np.matrix(np.identity(3)),
                    "scale": np.matrix(np.identity(3))
                },
                log_prefix_str="    ")

    mayavi.mlab.show()


def draw_components(component_desc, world_from_component_transform_func, world_from_component_transform_data, log_prefix_str=""):

    # Only process SceneComponents...
    component_class = component_desc["class"]
    if component_class in scene_component_classes:

        spear.log(log_prefix_str, "Processing SceneComponent: ", component_desc["name"])
        M_world_from_component, world_from_component_transform_data = \
            world_from_component_transform_func(component_desc, world_from_component_transform_data)

        # ...and only attempt to draw StaticMeshComponents...
        if component_class in static_mesh_component_classes:
            spear.log(log_prefix_str, "    Component is a StaticMeshComponent.")
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]

            # ...that refer to non-null StaticMesh assets...
            if static_mesh_desc is not None:
                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
                spear.log(log_prefix_str, "    StaticMesh asset path: ", static_mesh_asset_path)
    
                # ...that are in the /Game/Scenes/<scene_id> directory.
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):

                    obj_path_suffix = posixpath.join(*static_mesh_asset_path.parts[4:]) + ".obj"
                    numerical_parity_obj_path = \
                        os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))

                    spear.log(log_prefix_str, "    OBJ file:              ", numerical_parity_obj_path)

                    mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)

                    V_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
                    V_world = M_world_from_component*V_component
                    assert np.allclose(V_world[3,:], 1.0)

                    mesh.vertices = V_world.T.A[:,0:3]

                    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                    if args.visual_parity_with_unreal:
                        mesh.vertices = mesh.vertices[:,[0,2,1]]

                    mayavi.mlab.triangular_mesh(
                        mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=c_face, opacity=mesh_opacity)

        # Recurse for each child component.
        for child_component_desc in component_desc["children_components"].values():
            draw_components(
                child_component_desc, world_from_component_transform_func, world_from_component_transform_data, log_prefix_str=log_prefix_str + "    ")


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
    # But these conventions conflict. We therefore need to negate Unreal's roll (rotation around X) and pitch
    # (rotation around Y) but not yaw (rotation around Z) when constructing a scipy.spatial.transform.Rotation
    # object from Unreal roll-pitch-yaw angles. Unreal editor properties also specify roll-pitch-yaw Euler
    # angles in degrees, scipy.spatial.transform.Rotation.from_euler(...) expects radians by default. So we
    # also need to convert from degrees to radians.
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

    # Construct the 4x4 world-from-component transformation matrix by applying transformation
    # in the following order,
    #     1. Scale
    #     2. Rotation
    #     3. Location 
    # as discussed here:
    #     https://docs.unrealengine.com/5.2/en-US/API/Runtime/Core/Math/FTransform

    M_l_world_from_component = np.matrix(np.block([[np.identity(3),         l_world_from_component], [np.zeros([1,3]), 1.0]]))
    M_R_world_from_component = np.matrix(np.block([[R_world_from_component, np.zeros([3,1])],        [np.zeros([1,3]), 1.0]]))
    M_S_world_from_component = np.matrix(np.block([[S_world_from_component, np.zeros([3,1])],        [np.zeros([1,3]), 1.0]]))

    M_world_from_component = M_l_world_from_component*M_R_world_from_component*M_S_world_from_component

    # Check the M_world_from_component matrix that we computed above against the default
    # SceneComponent.get_world_transform() function. We store the output from this function
    # in our exported JSON file for each SceneComponent, so we can simply compare the matrix
    # we computed above against the stored matrix. Note that each "plane" below refers to a
    # column, so in this sense, editor properties represent matrices in column-major order.

    M_world_from_component_00_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["x"]
    M_world_from_component_10_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["y"]
    M_world_from_component_20_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["z"]
    M_world_from_component_30_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["w"]
    M_world_from_component_01_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["x"]
    M_world_from_component_11_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["y"]
    M_world_from_component_21_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["z"]
    M_world_from_component_31_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["w"]
    M_world_from_component_02_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["x"]
    M_world_from_component_12_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["y"]
    M_world_from_component_22_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["z"]
    M_world_from_component_32_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["w"]
    M_world_from_component_03_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["x"]
    M_world_from_component_13_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["y"]
    M_world_from_component_23_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["z"]
    M_world_from_component_33_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["w"]

    M_world_from_component_ = np.matrix([
        [M_world_from_component_00_, M_world_from_component_01_, M_world_from_component_02_, M_world_from_component_03_],
        [M_world_from_component_10_, M_world_from_component_11_, M_world_from_component_12_, M_world_from_component_13_],
        [M_world_from_component_20_, M_world_from_component_21_, M_world_from_component_22_, M_world_from_component_23_],
        [M_world_from_component_30_, M_world_from_component_31_, M_world_from_component_32_, M_world_from_component_33_]])
    assert np.allclose(M_world_from_component, M_world_from_component_)

    world_from_component_transform_data = {"location": l_world_from_component, "rotation": R_world_from_component, "scale": S_world_from_component}

    return M_world_from_component, world_from_component_transform_data


if __name__ == '__main__':
    process_scene()
