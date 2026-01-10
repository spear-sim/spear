#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import json
import numpy as np
import os
import pathlib
import scipy

identity_transform = {"location": np.matrix(np.zeros(3)).T, "rotation": np.matrix(np.identity(3)), "scale": np.matrix(np.identity(3))}
matrix_column_names = ["x_plane","y_plane","z_plane", "w_plane"]
matrix_row_names = ["x", "y", "z", "w"]

#
# Functions for working with components.
#

def compose_transform_with_component(transform_ancestor_from_parent_component, component_desc):

    absolute_location = component_desc["editor_properties"]["absolute_location"]
    absolute_rotation = component_desc["editor_properties"]["absolute_rotation"]
    absolute_scale    = component_desc["editor_properties"]["absolute_scale"]

    relative_location_x     = component_desc["editor_properties"]["relative_location"]["editor_properties"]["x"]
    relative_location_y     = component_desc["editor_properties"]["relative_location"]["editor_properties"]["y"]
    relative_location_z     = component_desc["editor_properties"]["relative_location"]["editor_properties"]["z"]
    relative_rotation_roll  = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["roll"]
    relative_rotation_pitch = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["pitch"]
    relative_rotation_yaw   = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["yaw"]
    relative_scale3d_x      = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["x"]
    relative_scale3d_y      = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["y"]
    relative_scale3d_z      = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["z"]

    #
    # Unreal defines each individual Euler angle according to the following conventions, which can be
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
    # object from an individual Unreal Euler angle. Unreal editor properties also specify Euler angles in
    # degrees, whereas the scipy.spatial.transform.Rotation.from_euler(...) function expects radians by default.
    # So we also need to convert from degrees to radians.
    #

    roll  = np.deg2rad(-relative_rotation_roll)
    pitch = np.deg2rad(-relative_rotation_pitch)
    yaw   = np.deg2rad(relative_rotation_yaw)

    # 
    # Additionally, Unreal applies a triplet of roll-pitch-yaw Euler angles in the fixed parent coordinate
    # system in the following order, which can be verified by manual inspection the editor.
    #     1. Rotate around fixed parent-space X by roll degrees
    #     2. Rotate around fixed parent-space Y by pitch degrees
    #     3. Rotate around fixed parent-space Z by yaw degrees
    # 
    # So, given a triplet of roll-pitch-yaw values that has been negated appropriately and converted to
    # radians as described above, we define the rotation matrix that corresponds to the roll-pitch-yaw values
    # as follows.
    #     R_x = np.matrix(scipy.spatial.transform.Rotation.from_euler("x", roll).as_matrix())
    #     R_y = np.matrix(scipy.spatial.transform.Rotation.from_euler("y", pitch).as_matrix())
    #     R_z = np.matrix(scipy.spatial.transform.Rotation.from_euler("z", yaw).as_matrix())
    #     R   = R_z*R_y*R_x
    # which is equivalent to the following expression,
    #     R   = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    #
    # Once we have computed R, we can use it to rotate a child-space point p_child into parent-space as follows.
    #     p_parent = R*p_child
    #

    transform_parent_component_from_current_component = {}
    transform_parent_component_from_current_component["location"] = np.matrix([relative_location_x, relative_location_y, relative_location_z]).T
    transform_parent_component_from_current_component["rotation"] = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    transform_parent_component_from_current_component["scale"] = np.matrix(np.diag([relative_scale3d_x, relative_scale3d_y, relative_scale3d_z]))

    transform_ancestor_from_current_component = compose_transforms(transforms=[transform_ancestor_from_parent_component, transform_parent_component_from_current_component])

    # If we're in absolute mode for {location, rotation, scale}, then don't accumulate.
    if absolute_location:
        transform_ancestor_from_current_component["location"] = transform_parent_component_from_current_component["location"]
    if absolute_rotation:
        transform_ancestor_from_current_component["rotation"] = transform_parent_component_from_current_component["rotation"]
    if absolute_scale:
        transform_ancestor_from_current_component["scale"] = transform_parent_component_from_current_component["scale"]

    return transform_ancestor_from_current_component

def any_component_transform_absolute(component_desc):

    absolute_location = component_desc["editor_properties"]["absolute_location"] 
    absolute_rotation = component_desc["editor_properties"]["absolute_rotation"] 
    absolute_scale    = component_desc["editor_properties"]["absolute_scale"]   

    return absolute_location or absolute_rotation or absolute_scale

#
# Functions for getting transform and transform data objects. Transform data objects are serializable as JSON.
#

def compose_transforms(transforms):

    l_composed = identity_transform["location"]
    R_composed = identity_transform["rotation"]
    S_composed = identity_transform["scale"]

    for transform in transforms[::-1]:
        l_current = transform["location"]
        R_current = transform["rotation"]
        S_current = transform["scale"]

        eps = 0.000001
        assert np.all(np.diag(S_current) > eps)

        # This formulation for accumulating {location, rotation, scale} through the component hierarchy is not
        # immediately obvious to me, but it matches the behavior of USceneComponent and FTransform, see:
        #     Engine/Source/Runtime/Engine/Private/Components/SceneComponent.cpp
        #     Engine/Source/Runtime/Core/Public/Math/TransformNonVectorized.h

        l_composed = R_current*S_current*l_composed + l_current
        R_composed = R_current*R_composed
        S_composed = S_composed*S_current  

    transform_composed = {"location": l_composed, "rotation": R_composed, "scale": S_composed}

    return transform_composed

def get_transform_from_transform_data(transform_data):

    if transform_data is None:
        return None

    transform = {}
    transform["location"] = get_matrix_from_matrix_data(matrix_data=transform_data["location"])
    transform["rotation"] = get_matrix_from_matrix_data(matrix_data=transform_data["rotation"])
    transform["scale"]    = get_matrix_from_matrix_data(matrix_data=transform_data["scale"])

    return transform

def get_transform_data_from_transform(transform):

    if transform is None:
        return None

    transform_data = {}
    transform_data["location"] = get_matrix_data_from_matrix(matrix=transform["location"])
    transform_data["rotation"] = get_matrix_data_from_matrix(matrix=transform["rotation"])
    transform_data["scale"]    = get_matrix_data_from_matrix(matrix=transform["scale"])

    return transform_data

#
# Functions for getting matrix and matrix data objects. Matrix data objects are serializable as JSON. Note that
# in these functions, each "plane" below refers to a matrix column. So in this sense, Unreal editor properties
# and our matrix data objects represent matrices in column-major order.
#

def get_matrix_from_transform(transform):

    # Construct a 4x4 transformation matrix by applying the input transformation components in the following
    # order: (1) scale; (2) rotation; (3) translation. See the following link for more details:
    #     https://docs.unrealengine.com/5.2/en-US/API/Runtime/Core/Math/FTransform

    M_l = np.matrix(np.block([[np.identity(3),        transform["location"]], [np.zeros([1,3]), 1.0]]))
    M_R = np.matrix(np.block([[transform["rotation"], np.zeros([3,1])],       [np.zeros([1,3]), 1.0]]))
    M_S = np.matrix(np.block([[transform["scale"],    np.zeros([3,1])],       [np.zeros([1,3]), 1.0]]))

    matrix = M_l*M_R*M_S

    return matrix

def get_matrix_from_matrix_data(matrix_data):

    if matrix_data is None:
        return None

    num_rows = 0
    num_columns = 0
    for column, column_name in enumerate(matrix_column_names):        
        if column_name in matrix_data:
            num_columns = np.maximum(num_columns, column+1)
            for row, row_name in enumerate(matrix_row_names):
                if row_name in matrix_data[column_name]:
                    num_rows = np.maximum(num_rows, row+1)

    matrix = np.nan*np.matrix(np.ones([num_rows, num_columns]))
    for row, row_name in enumerate(matrix_row_names[:num_rows]):
        for column, column_name in enumerate(matrix_column_names[:num_columns]):
            matrix[row, column] = matrix_data[column_name][row_name]
    assert np.all(np.isfinite(matrix))

    return matrix

def get_matrix_from_matrix_desc(matrix_desc):

    if matrix_desc is None:
        return None

    num_rows = 4
    num_columns = 4

    matrix = np.nan*np.matrix(np.ones([num_rows, num_columns]))
    for row, row_name in enumerate(matrix_row_names):
        for column, column_name in enumerate(matrix_column_names):
            matrix[row, column] = matrix_desc["editor_properties"][column_name]["editor_properties"][row_name]
    assert np.all(np.isfinite(matrix))

    return matrix

def get_matrix_data_from_matrix(matrix):

    if matrix is None:
        return None

    assert len(matrix.shape) == 2
    assert matrix.shape[0] <= 4
    assert matrix.shape[1] <= 4

    num_columns = matrix.shape[1]
    num_rows = matrix.shape[0]

    matrix_data = {}
    for column, column_name in enumerate(matrix_column_names[:num_columns]):
        matrix_data[column_name] = {}
        for row, row_name in enumerate(matrix_row_names[:num_rows]):
            matrix_data[column_name][row_name] = matrix[row, column]

    return matrix_data

#
# Function for getting a physical filesystem path from a logical content path.
#

def get_filesystem_path_from_content_path(content_path, unreal_project_dir, unreal_engine_dir):

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        filesystem_base_dir = os.path.join(unreal_project_dir, "Content")
    elif content_root == "Engine":
        filesystem_base_dir = os.path.join(unreal_engine_dir, "Engine", "Content")
    else:
        plugins_dirs = [
            os.path.join(unreal_project_dir, "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Experimental")]

        uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
        assert len(uprojects) == 1
        uproject = uprojects[0]
        uproject_dict = {}
        with open(uproject) as f:
            uproject_dict = json.load(f)
        if "AdditionalPluginDirectories" in uproject_dict:
            for additional_plugins_dir in uproject_dict["AdditionalPluginDirectories"]:
                if os.path.isabs(additional_plugins_dir):
                    plugins_dirs.append(additional_plugins_dir)
                else:
                    plugins_dirs.append(os.path.realpath(os.path.join(unreal_project_dir, additional_plugins_dir)))

        found_plugin = False
        for plugins_dir in plugins_dirs:
            plugin_dir = os.path.realpath(os.path.join(plugins_dir, content_root))
            if os.path.exists(plugin_dir):
                filesystem_base_dir = os.path.realpath(os.path.join(plugin_dir, "Content"))
                found_plugin = True
                break
        assert found_plugin

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = os.path.realpath(os.path.join(filesystem_base_dir, content_sub_path))
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) == 1:
                filesystem_paths = glob.glob(os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            elif len(content_file_tokens) == 2:
                assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            else:
                assert False
