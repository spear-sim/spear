#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

#
# NOTES ON UNREAL'S CONVENTIONS FOR REPRESENTING ROTATIONS AND TRANSFORMING COORDINATES
#
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
#     roll  = np.deg2rad(-unreal_roll)
#     pitch = np.deg2rad(-unreal_pitch)
#     yaw   = np.deg2rad(unreal_yaw)
#
# Additionally, Unreal applies a triplet of roll-pitch-yaw Euler angles in the fixed parent coordinate
# system in the following order, which can be verified by manual inspection the editor.
#     1. Rotate around fixed parent-space X by roll degrees
#     2. Rotate around fixed parent-space Y by pitch degrees
#     3. Rotate around fixed parent-space Z by yaw degrees
#
# So, given a triplet of roll-pitch-yaw values that has been negated appropriately and converted to
# radians as described above, we define the rotation matrix that corresponds to our derived roll-pitch-yaw
# values as follows.
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
# Finally, Unreal uses the following convention for accumulating {location, rotation, scale} through its
# component hierarchy. The motivation for this particular convention is not immediately obvious to me, but
# it matches the behavior of USceneComponent and FTransform, see:
#     Engine/Source/Runtime/Engine/Private/Components/SceneComponent.cpp
#     Engine/Source/Runtime/Core/Public/Math/TransformNonVectorized.h
#
#     l_composed = zero_column_vector
#     R_composed = identity_matrix
#     S_composed = diagonal_scale_matrix
#
#     for transform in transforms_in_leaf_to_root_order:
#         l_current = transform["location"]
#         R_current = transform["rotation"]
#         S_current = transform["scale"]
#     
#         l_composed = R_current*S_current*l_composed + l_current
#         R_composed = R_current*R_composed
#         S_composed = S_composed*S_current
#

import numpy as np
import scipy


#
# Math conversion functions. In the functions below, converting to or from a "spear" type means a
# dictionary that is returned when calling a UFUNCTION or accessing a UPROPERTY via the SPEAR API.
#

# FMatrix

def to_numpy_matrix_from_spear_matrix(spear_matrix, as_matrix=None):
    assert isinstance(spear_matrix, dict)
    spear_matrix = { k.lower(): v for k, v in spear_matrix.items() }
    assert set(["xplane", "yplane", "zplane", "wplane"]) == set(spear_matrix.keys())
    numpy_matrix = np.column_stack([ # each plane is a column
        to_numpy_array_from_spear_plane(spear_plane=spear_matrix["xplane"]),
        to_numpy_array_from_spear_plane(spear_plane=spear_matrix["yplane"]),
        to_numpy_array_from_spear_plane(spear_plane=spear_matrix["zplane"]),
        to_numpy_array_from_spear_plane(spear_plane=spear_matrix["wplane"])])
    if as_matrix is None:
        return numpy_matrix
    else:
        assert as_matrix
        return np.matrix(numpy_matrix)

def to_spear_matrix_from_numpy_matrix(numpy_matrix):
    if isinstance(numpy_matrix, np.matrix):
        numpy_matrix = numpy_matrix.A
    if isinstance(numpy_matrix, np.ndarray):
        assert numpy_matrix.shape == (4, 4)
    else:
        assert False
    return { # each plane is a column
        "XPlane": to_spear_plane_from_numpy_array(numpy_array=numpy_matrix[:, 0]),
        "YPlane": to_spear_plane_from_numpy_array(numpy_array=numpy_matrix[:, 1]),
        "ZPlane": to_spear_plane_from_numpy_array(numpy_array=numpy_matrix[:, 2]),
        "WPlane": to_spear_plane_from_numpy_array(numpy_array=numpy_matrix[:, 3])}

# FPlane

def to_numpy_array_from_spear_plane(spear_plane, as_matrix=None):
    assert isinstance(spear_plane, dict)
    spear_plane = { k.lower(): v for k, v in spear_plane.items() }
    assert set(["x", "y", "z", "w"]) == set(spear_plane.keys())
    numpy_array = np.array([spear_plane["x"], spear_plane["y"], spear_plane["z"], spear_plane["w"]])
    if as_matrix is None:
        return numpy_array
    else:
        assert as_matrix
        return np.matrix(numpy_array).T

def to_spear_plane_from_numpy_array(numpy_array):
    if isinstance(numpy_array, np.matrix):
        numpy_array = numpy_array.A1
    if isinstance(numpy_array, np.ndarray):
        assert numpy_array.shape == (4,)
    else:
        assert False
    return {"X": float(numpy_array[0]), "Y": float(numpy_array[1]), "Z": float(numpy_array[2]), "W": float(numpy_array[3])}

# FQuat

def to_numpy_array_from_spear_quat(spear_quat):
    assert isinstance(spear_quat, dict)
    spear_quat = { k.lower(): v for k, v in spear_quat.items() }
    assert set(["x", "y", "z", "w"]) == set(spear_quat.keys())
    return np.array([spear_quat["x"], spear_quat["y"], spear_quat["z"], spear_quat["w"]]) # scipy.spatial.transform.Rotation assumes scalar-last (xyzw) order so we follow that convention here

def to_numpy_matrix_from_spear_quat(spear_quat, as_matrix=None):
    numpy_array_xyzw = to_numpy_array_from_spear_quat(spear_quat=spear_quat)
    numpy_matrix = np.array(scipy.spatial.transform.Rotation.from_quat(numpy_array_xyzw).as_matrix())
    if as_matrix is None:
        return numpy_matrix
    else:
        assert as_matrix
        return np.matrix(numpy_matrix)

def to_spear_quat_from_numpy_array(numpy_array_xyzw):
    if isinstance(numpy_array_xyzw, np.ndarray):
        assert numpy_array_xyzw.shape == (4,)
    else:
        assert False
    return {"X": float(numpy_array_xyzw[0]), "Y": float(numpy_array_xyzw[1]), "Z": float(numpy_array_xyzw[2]), "W": float(numpy_array_xyzw[3])} # assumes scalar-last (xyzw) order

def to_spear_quat_from_numpy_matrix(numpy_matrix):
    if isinstance(numpy_matrix, np.matrix):
        numpy_matrix = numpy_matrix.A
    if isinstance(numpy_matrix, np.ndarray):
        assert numpy_matrix.shape == (3, 3)
    else:
        assert False
    numpy_array_xyzw = scipy.spatial.transform.Rotation.from_matrix(numpy_matrix).as_quat()
    return to_spear_quat_from_numpy_array(numpy_array_xyzw=numpy_array_xyzw)

# FRotator

def to_numpy_array_from_spear_rotator(spear_rotator):
    assert isinstance(spear_rotator, dict)
    spear_rotator = { k.lower(): v for k, v in spear_rotator.items() }
    assert set(["roll", "pitch", "yaw"]) == set(spear_rotator.keys())
    return np.array([spear_rotator["pitch"], spear_rotator["yaw"], spear_rotator["roll"]]) # assumes pyr order

def to_numpy_matrix_from_spear_rotator(spear_rotator, as_matrix=None):
    assert isinstance(spear_rotator, dict)
    spear_rotator = { k.lower(): v for k, v in spear_rotator.items() }
    assert set(["roll", "pitch", "yaw"]) == set(spear_rotator.keys())
    roll  = np.deg2rad(-spear_rotator["roll"])
    pitch = np.deg2rad(-spear_rotator["pitch"])
    yaw   = np.deg2rad(spear_rotator["yaw"])
    numpy_matrix = scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    if as_matrix is None:
        return numpy_matrix
    else:
        assert as_matrix
        return np.matrix(numpy_matrix)

def to_spear_rotator_from_numpy_array(numpy_array_pyr):
    if isinstance(numpy_array_pyr, np.ndarray):
        assert numpy_array_pyr.shape == (3,)
    else:
        assert False
    return {"Pitch": float(numpy_array_pyr[0]), "Yaw": float(numpy_array_pyr[1]), "Roll": float(numpy_array_pyr[2])}

def to_spear_rotator_from_numpy_matrix(numpy_matrix):
    if isinstance(numpy_matrix, np.ndarray) or isinstance(numpy_matrix, np.matrix):
        assert numpy_matrix.shape == (3,3)
    else:
        assert False
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_matrix(numpy_matrix).as_euler("xyz")
    roll  = float(np.rad2deg(-scipy_roll))
    pitch = float(np.rad2deg(-scipy_pitch))
    yaw   = float(np.rad2deg(scipy_yaw))
    return {"Roll": roll, "Pitch": pitch, "Yaw": yaw}

# FTransform

def to_numpy_transform_from_spear_transform(spear_transform, as_quat=None, as_array=None, as_matrix=None):
    return convert_transform(transform=spear_transform, as_numpy=True, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)

def to_numpy_matrix_from_spear_transform(spear_transform, as_matrix=None):
    numpy_transform = to_numpy_transform_from_spear_transform(spear_transform=spear_transform, as_matrix=True)
    M_t = np.matrix(np.block([[np.identity(3),              numpy_transform["translation"]], [np.zeros((1, 3)), 1.0]]))
    M_R = np.matrix(np.block([[numpy_transform["rotation"], np.zeros((3, 1))],               [np.zeros((1, 3)), 1.0]]))
    M_S = np.matrix(np.block([[numpy_transform["scale"],    np.zeros((3, 1))],               [np.zeros((1, 3)), 1.0]]))
    M = M_t*M_R*M_S
    if as_matrix is None:
        return M.A
    else:
        assert as_matrix
        return M

def to_spear_transform_from_numpy_transform(numpy_transform):
    return convert_transform(transform=numpy_transform, as_spear=True)

def to_spear_transform_from_numpy_matrix(numpy_matrix):
    if isinstance(numpy_matrix, np.matrix):
        numpy_matrix = numpy_matrix.A
    if isinstance(numpy_matrix, np.ndarray):
        assert numpy_matrix.shape == (4, 4)
    else:
        assert False
    t = numpy_matrix[:3, 3]
    M_RS = numpy_matrix[:3, :3]
    s = np.linalg.norm(M_RS, axis=0)
    M_R = M_RS/s

    # If the determinant of M_R is negative, the matrix includes a reflection. Absorb the reflection
    # into the scale (by negating the X axis), so that M_R is a proper rotation matrix. This approach
    # matches the behavior of FTransform::SetFromMatrix(...), see:
    #     Engine/Source/Runtime/Core/Public/Math/TransformNonVectorized.h

    if np.linalg.det(M_R) < 0:
        s[0] = -s[0]
        M_R[:, 0] = -M_R[:, 0]

    translation = to_spear_vector_from_numpy_array(numpy_array=t)
    rotation = to_spear_rotator_from_numpy_matrix(numpy_matrix=M_R)
    scale_3d = to_spear_vector_from_numpy_array(numpy_array=s)
    return {"Translation": translation, "Rotation": rotation, "Scale3D": scale_3d}

# FVector

def to_numpy_array_from_spear_vector(spear_vector, as_matrix=None):
    assert isinstance(spear_vector, dict)
    spear_vector = { k.lower(): v for k, v in spear_vector.items() }
    assert set(["x", "y", "z"]) == set(spear_vector.keys())
    if as_matrix is None:
        return np.array([spear_vector["x"], spear_vector["y"], spear_vector["z"]])
    else:
        assert as_matrix
        return np.matrix([spear_vector["x"], spear_vector["y"], spear_vector["z"]]).T

def to_spear_vector_from_numpy_array(numpy_array):
    if isinstance(numpy_array, np.matrix):
        assert numpy_array.shape == (3, 1)
        numpy_array = numpy_array.A1
    elif isinstance(numpy_array, np.ndarray):
        assert numpy_array.shape == (3,)
    else:
        assert False
    return {"X": float(numpy_array[0]), "Y": float(numpy_array[1]), "Z": float(numpy_array[2])}


#
# Convert between transform representations.
#

def convert_transform(transform, as_numpy=None, as_spear=None, as_quat=None, as_array=None, as_matrix=None):

    # validate input

    assert isinstance(transform, dict)
    if as_numpy is not None:
        assert as_numpy
        assert as_spear is None
    if as_spear is not None:
        assert as_spear
        assert as_numpy is None
        assert as_quat is None
        assert as_array is None
        assert as_matrix is None
    if as_quat is not None:
        assert as_quat
        assert as_array is None
        assert as_matrix is None
    if as_array is not None:
        assert as_array
        assert as_matrix is None

    # normalize input to a numpy transform with a (3,) np.ndarray translation vector, a (3,3) np.ndarray
    # rotation matrix, and a (3,) np.ndarray scale vector

    keys = set(transform.keys())
    if keys == set(["translation", "rotation", "scale"]):
        numpy_transform = transform
    elif set(k.lower() for k in keys) == set(["translation", "rotation", "scale3d"]):
        spear_transform = { k.lower(): v for k, v in transform.items() }
        numpy_transform = {
            "translation": to_numpy_array_from_spear_vector(spear_vector=spear_transform["translation"]),
            "rotation": to_numpy_matrix_from_spear_quat(spear_quat=spear_transform["rotation"]),
            "scale": to_numpy_array_from_spear_vector(spear_vector=spear_transform["scale3d"])}
    else:
        assert False

    translation = numpy_transform["translation"]
    if isinstance(translation, np.matrix):
        translation = translation.A1
    if isinstance(translation, np.ndarray):
        assert translation.shape == (3,)
    else:
        assert False

    rotation = numpy_transform["rotation"]
    if isinstance(rotation, np.matrix):
        rotation = rotation.A
    if isinstance(rotation, np.ndarray):
        if rotation.shape == (4,):
            rotation = scipy.spatial.transform.Rotation.from_quat(rotation).as_matrix()
        elif rotation.shape == (3, 3):
            pass
        else:
            assert False
    else:
        assert False

    scale = numpy_transform["scale"]
    if isinstance(scale, np.matrix):
        assert scale.shape == (3,3)
        assert np.all(scale == np.diag(np.diag(scale)))
        scale = np.diag(scale)
    if isinstance(scale, np.ndarray):
        assert scale.shape == (3,)
    else:
        assert False

    # format output

    if as_numpy:
        if as_quat is not None:
            rotation = scipy.spatial.transform.Rotation.from_matrix(rotation).as_quat()
        elif as_array is not None:
            pass
        elif as_matrix is not None:
            translation = np.matrix(translation).T
            rotation = np.matrix(rotation)
            scale = np.matrix(np.diag(scale))
        else:
            rotation = scipy.spatial.transform.Rotation.from_matrix(rotation).as_quat()
        return {"translation": translation, "rotation": rotation, "scale": scale}
    elif as_spear:
        translation = to_spear_vector_from_numpy_array(numpy_array=translation)
        rotation = to_spear_quat_from_numpy_matrix(numpy_matrix=rotation)
        scale = to_spear_vector_from_numpy_array(numpy_array=scale)
        return {"Translation": translation, "Rotation": rotation, "Scale3D": scale}
    else:
        assert False


#
# Composition functions
#

# Compose an array of transforms given in root-to-leaf order.
def compose_transforms(transforms, as_numpy=None, as_spear=None, as_quat=None, as_array=None, as_matrix=None):
    numpy_transforms = [ convert_transform(transform=t, as_numpy=True, as_matrix=True) for t in transforms ]

    t_composed = np.matrix(np.zeros(3)).T
    R_composed = np.matrix(np.identity(3))
    S_composed = np.matrix(np.identity(3))

    for numpy_transform in numpy_transforms[::-1]:
        t_current = numpy_transform["translation"]
        R_current = numpy_transform["rotation"]
        S_current = numpy_transform["scale"]

        t_composed = R_current*S_current*t_composed + t_current
        R_composed = R_current*R_composed
        S_composed = S_composed*S_current

    transform_composed = {"translation": t_composed, "rotation": R_composed, "scale": S_composed}
    return convert_transform(transform=transform_composed, as_numpy=as_numpy, as_spear=as_spear, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)

# Compose an array of transforms given in root-to-leaf order, respecting per-transform absolute flags.
def compose_component_transforms(transforms, is_absolute_location=None, is_absolute_rotation=None, is_absolute_scale=None, as_numpy=None, as_spear=None, as_quat=None, as_array=None, as_matrix=None):

    num_transforms = len(transforms)
    assert num_transforms >= 1

    def _broadcast(flags, default=False):
        if flags is None:
            return [default]*(num_transforms - 1)
        elif isinstance(flags, bool):
            return [flags]*(num_transforms - 1)
        else:
            assert len(flags) == num_transforms - 1
            return flags

    is_absolute_location = _broadcast(is_absolute_location)
    is_absolute_rotation = _broadcast(is_absolute_rotation)
    is_absolute_scale = _broadcast(is_absolute_scale)

    transform_composed = transforms[0]
    for i in range(1, num_transforms):
        is_absolute_location_current = is_absolute_location[i - 1]
        is_absolute_rotation_current = is_absolute_rotation[i - 1]
        is_absolute_scale_current = is_absolute_scale[i - 1]

        if is_absolute_location_current and is_absolute_rotation_current and is_absolute_scale_current:
            transform_composed = transforms[i]
        elif is_absolute_location_current or is_absolute_rotation_current or is_absolute_scale_current:
            transform_composed = compose_transforms(transforms=[transform_composed, transforms[i]], as_numpy=True, as_array=True)
            child = convert_transform(transform=transforms[i], as_numpy=True, as_array=True)
            if is_absolute_location_current:
                transform_composed["translation"] = child["translation"]
            if is_absolute_rotation_current:
                transform_composed["rotation"] = child["rotation"]
            if is_absolute_scale_current:
                transform_composed["scale"] = child["scale"]
        else:
            transform_composed = compose_transforms(transforms=[transform_composed, transforms[i]], as_numpy=True, as_array=True)

    return convert_transform(transform=transform_composed, as_numpy=as_numpy, as_spear=as_spear, as_quat=as_quat, as_array=as_array, as_matrix=as_matrix)


#
# Identity transform
#

identity_transform = to_spear_transform_from_numpy_transform(numpy_transform={"translation": np.zeros(3), "rotation": np.identity(3), "scale": np.ones(3)})
