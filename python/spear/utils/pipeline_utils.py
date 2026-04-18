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
import spear


#
# Math conversion functions. In the functions below, converting to or from a JSON type means a dictionary
# that has been serialized to JSON by pipeline/export_unreal_metadata/run.py and subsequentially deserialized
# back into a dictionary.
#

# JSON matrix

def to_numpy_matrix_from_json_matrix(json_matrix, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_matrix(spear_matrix=to_spear_matrix_from_json_matrix(json_matrix=json_matrix), as_matrix=as_matrix)

# JSON plane

def to_numpy_array_from_json_plane(json_plane, as_matrix=None):
    return spear.math.to_numpy_array_from_spear_plane(spear_plane=to_spear_plane_from_json_plane(json_plane=json_plane), as_matrix=None)

# JSON quaternion

def to_numpy_array_from_json_quat(json_quat):
    return spear.math.to_numpy_array_from_spear_quat(spear_quat=to_spear_quat_from_json_quat(json_quat=json_quat))

def to_numpy_matrix_from_json_quat(json_quat, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_quat(spear_quat=to_spear_quat_from_json_quat(json_quat=json_quat), as_matrix=as_matrix)

# JSON rotator

def to_numpy_array_from_json_rotator(json_rotator):
    return spear.math.to_numpy_array_from_spear_rotator(spear_rotator=to_spear_rotator_from_json_rotator(json_rotator=json_rotator))

def to_numpy_matrix_from_json_rotator(json_rotator, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=to_spear_rotator_from_json_rotator(json_rotator=json_rotator), as_matrix=as_matrix)

# JSON transform

def to_numpy_dict_from_json_transform(json_transform, as_quat=None, as_array=None, as_matrix=None):
    return spear.math.to_numpy_transform_from_spear_transform(spear_transform=to_spear_transform_from_json_transform(json_transform=json_transform), as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)

def to_numpy_matrix_from_json_transform(json_transform, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_transform(spear_transform=to_spear_transform_from_json_transform(json_transform=json_transform), as_matrix=as_matrix)

# JSON vector

def to_numpy_array_from_json_vector(json_vector, as_matrix=None):
    return spear.math.to_numpy_array_from_spear_vector(spear_vector=to_spear_vector_from_json_vector(json_vector=json_vector), as_matrix=as_matrix)


#
# Helper functions for converting to and from JSON dictionaries.
#

# JSON matrix

def to_spear_matrix_from_json_matrix(json_matrix):
    return {
        "XPlane": to_spear_plane_from_json_plane(json_plane=json_matrix["editor_properties"]["x_plane"]),
        "YPlane": to_spear_plane_from_json_plane(json_plane=json_matrix["editor_properties"]["y_plane"]),
        "ZPlane": to_spear_plane_from_json_plane(json_plane=json_matrix["editor_properties"]["z_plane"]),
        "WPlane": to_spear_plane_from_json_plane(json_plane=json_matrix["editor_properties"]["w_plane"])}

# JSON plane

def to_spear_plane_from_json_plane(json_plane):
    return {"X": json_plane["editor_properties"]["x"], "Y": json_plane["editor_properties"]["y"], "Z": json_plane["editor_properties"]["z"], "W": json_plane["editor_properties"]["w"]}

# JSON quaternion

def to_spear_quat_from_json_quat(json_quat):
    return to_spear_plane_from_json_plane(json_plane=json_quat)

# JSON rotator

def to_spear_rotator_from_json_rotator(json_rotator):
    return {"Roll": json_rotator["editor_properties"]["roll"], "Pitch": json_rotator["editor_properties"]["pitch"], "Yaw": json_rotator["editor_properties"]["yaw"]}

# JSON transform

def to_spear_transform_from_json_transform(json_transform):
    return {
        "Translation": to_spear_vector_from_json_vector(json_vector=json_transform["editor_properties"]["translation"]),
        "Rotation": to_spear_quat_from_json_quat(json_quat=json_transform["editor_properties"]["rotation"]),
        "Scale3D": to_spear_vector_from_json_vector(json_vector=json_transform["editor_properties"]["scale3d"])}

def to_spear_transform_from_json_component(json_component):
    rotation_matrix = to_numpy_matrix_from_json_rotator(json_rotator=json_component["editor_properties"]["relative_rotation"])
    transform = {
        "Translation": to_spear_vector_from_json_vector(json_vector=json_component["editor_properties"]["relative_location"]),
        "Rotation": spear.math.to_spear_quat_from_numpy_matrix(numpy_matrix=rotation_matrix),
        "Scale3D": to_spear_vector_from_json_vector(json_vector=json_component["editor_properties"]["relative_scale3d"])}
    is_absolute_location = json_component["editor_properties"]["absolute_location"]
    is_absolute_rotation = json_component["editor_properties"]["absolute_rotation"]
    is_absolute_scale = json_component["editor_properties"]["absolute_scale"]
    return transform, is_absolute_location, is_absolute_rotation, is_absolute_scale

# JSON vector

def to_spear_vector_from_json_vector(json_vector):
    return {"X": json_vector["editor_properties"]["x"], "Y": json_vector["editor_properties"]["y"], "Z": json_vector["editor_properties"]["z"]}


#
# Function for getting a physical filesystem path from a logical content path.
#

def get_filesystem_path_from_content_path(content_path, unreal_project_dir=None, unreal_engine_dir=None, follow_symlinks=False):

    if follow_symlinks:
        path_func = lambda path: os.path.realpath(path)
    else:
        path_func = lambda path: path

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        assert unreal_project_dir is not None
        filesystem_base_dir = os.path.join(unreal_project_dir, "Content")

    elif content_root == "Engine":
        assert unreal_engine_dir is not None
        filesystem_base_dir = os.path.join(unreal_engine_dir, "Engine", "Content")

    else:
        assert unreal_engine_dir is not None
        assert unreal_project_dir is not None

        plugins_dirs = [
            os.path.join(unreal_project_dir, "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Experimental")]

        uprojects = glob.glob(path_func(os.path.join(unreal_project_dir, "*.uproject")))
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
                    plugins_dirs.append(path_func(os.path.join(unreal_project_dir, additional_plugins_dir)))

        found_plugin = False
        for plugins_dir in plugins_dirs:
            plugin_dir = path_func(os.path.join(plugins_dir, content_root))
            if os.path.exists(plugin_dir):
                filesystem_base_dir = path_func(os.path.join(plugin_dir, "Content"))
                found_plugin = True
                break
        assert found_plugin

    # try to follow Unreal's rules for resolving asset paths

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = path_func(os.path.join(filesystem_base_dir, content_sub_path))
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        elif os.path.islink(filesystem_path) and os.path.exists(os.path.realpath(filesystem_path)):
            spear.log("hello")
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) in [1, 2]:
                if len(content_file_tokens) == 2:
                    assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(path_func(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return path_func(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            else:
                assert False
