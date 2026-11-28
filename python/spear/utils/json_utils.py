#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


#
# Math conversion functions. In the functions below, converting to or from a JSON type means a dictionary
# that has been serialized to JSON by editor/export_scene_metadata.py and subsequently deserialized
# back into a dictionary.
#

# JSON matrix

def to_numpy_matrix_from_json_matrix(json_matrix, as_matrix=None):
    return spear.math.to_numpy_matrix_from_spear_matrix(spear_matrix=to_spear_matrix_from_json_matrix(json_matrix=json_matrix), as_matrix=as_matrix)

# JSON plane

def to_numpy_array_from_json_plane(json_plane, as_matrix=None):
    return spear.math.to_numpy_array_from_spear_plane(spear_plane=to_spear_plane_from_json_plane(json_plane=json_plane), as_matrix=as_matrix)

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

def to_numpy_transform_from_json_transform(json_transform, as_quat=None, as_array=None, as_matrix=None):
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
