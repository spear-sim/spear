#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import scipy
import spear
import scipy.spatial


def unreal_rpy_from_mujoco_quaternion(mujoco_quaternion):
    # MuJoCo assumes quaternions are stored in scalar-first (wxyz) order, but scipy.spatial.transform.Rotation assumes scalar-last (xyzw) order
    scipy_quaternion = mujoco_quaternion[[1, 2, 3, 0]]

    # Unreal and scipy.spatial.transform.Rotation have different Euler angle conventions, see python/spear/pipeline.py for details
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_quat(scipy_quaternion).as_euler("xyz")
    unreal_roll = np.rad2deg(-scipy_roll)
    unreal_pitch = np.rad2deg(-scipy_pitch)
    unreal_yaw = np.rad2deg(scipy_yaw)

    return np.array([unreal_roll, unreal_pitch, unreal_yaw])


# see https://github.com/google-deepmind/mujoco/blob/195bd32aa6bd9361245e0832f22651df7c44e81d/src/engine/engine_vis_visualize.c#L2133
def compute_camera_transform(cam):
    ca = np.cos(cam.azimuth / 180.0 * np.pi)
    sa = np.sin(cam.azimuth / 180.0 * np.pi)
    ce = np.cos(cam.elevation / 180.0 * np.pi)
    se = np.sin(cam.elevation / 180.0 * np.pi)
    forward = np.array([ce * ca, ce * sa, se])
    cam_location = np.array(cam.lookat) + forward * (-cam.distance)
    cam_rotation = np.array([0, cam.elevation, cam.azimuth])
    return cam_location.tolist(), cam_rotation.tolist()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf_file", default=r"F:\intel\interiorsim\pipeline\data\fetch_in_apartment_0000.mjcf")
    parser.add_argument("--agent", default=False)
    args = parser.parse_args()

    # create spear Instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    # initialize unreal
    with instance.begin_frame():
        # get Unreal uclasses
        actor_uclass = instance.unreal_service.get_static_class("AActor")
        static_mesh_component_uclass = instance.unreal_service.get_static_class("UStaticMeshComponent")
        pawn_uclass = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Script/CoreUObject.Class'/Script/Engine.Pawn'")
        urdf_uclass = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Agents/BP_Fetch.BP_Fetch_C")

        # get unreal functions
        set_actor_location_and_rotation_func = instance.unreal_service.find_function_by_name(uclass=actor_uclass,
                                                                                             function_name="K2_SetActorLocationAndRotation")
        set_component_relative_location_and_rotation_func = instance.unreal_service.find_function_by_name(uclass=static_mesh_component_uclass,
                                                                                                          function_name="K2_SetRelativeLocationAndRotation")
        set_component_world_location_and_rotation_func = instance.unreal_service.find_function_by_name(uclass=static_mesh_component_uclass,
                                                                                                       function_name="K2_SetWorldLocationAndRotation")
        # get all actors
        unreal_actors = instance.unreal_service.find_actors_by_type_as_dict(class_name="AActor")
        unreal_static_mesh_actors = instance.unreal_service.find_actors_by_type_as_dict(class_name="AStaticMeshActor")
        unreal_articulated_actors = {unreal_actor_name: unreal_actor for unreal_actor_name, unreal_actor in unreal_actors.items() if
                                     unreal_actor_name not in unreal_static_mesh_actors}

        # get all articulated components
        unreal_articulated_actor_components = {}
        for unreal_actor_name, unreal_actor in unreal_articulated_actors.items():
            unreal_articulated_actor_components[unreal_actor_name] = instance.unreal_service.get_components_by_type_as_dict("UStaticMeshComponent", unreal_actor)

        # get default pawn to synchronize unreal viewport
        default_pawn = instance.unreal_service.find_actor_by_class(uclass=pawn_uclass)

    with instance.end_frame():
        pass

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # get MuJoCo bodies
    mj_bodies = {mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if True}

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    # initialize MuJoCo camera (not needed when launching the viewer through the command-line, but needed when using launch_passive)
    mj_viewer.cam.distance = 30.0 * 100.0  # 30 meters * 100 Unreal units per meter
    mj_viewer.cam.azimuth = 90.0
    mj_viewer.cam.elevation = -45.0
    mj_viewer.cam.lookat = np.array([0.0, 0.0, 0.0])

    # initialize MuJoCo viewer options
    mj_viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION

    # update MuJoCo viewer state
    mj_viewer.sync()

    while mj_viewer.is_running():
        # perform multiple MuJoCo simulation steps per Unreal frame
        unreal_update_steps = 1
        for _ in range(unreal_update_steps):
            mj_update_steps = 10
            for _ in range(mj_update_steps):
                mujoco.mj_step(mj_model, mj_data)
            mj_viewer.sync()
        # get updated poses from MuJoCo
        mj_bodies_xpos = {mj_body_name: mj_data.body(mj_body).xpos for mj_body_name, mj_body in mj_bodies.items()}
        mj_bodies_xquat = {mj_body_name: mj_data.body(mj_body).xquat for mj_body_name, mj_body in mj_bodies.items()}

        # synchronize mujoco physics to unreal
        with instance.begin_frame():
            # update default pawn
            cam_location, cam_rotation = compute_camera_transform(mj_viewer.cam)
            instance.unreal_service.call_function(default_pawn, set_actor_location_and_rotation_func, {
                "NewLocation": dict(zip(["X", "Y", "Z"], cam_location)),
                "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], cam_rotation)),
                "bSweep": False,
                "bTeleport": True})

            # update static_mesh_actors
            for unreal_actor_name, unreal_actor in unreal_static_mesh_actors.items():
                instance.unreal_service.call_function(unreal_actor, set_actor_location_and_rotation_func, {
                    "NewLocation": dict(zip(["X", "Y", "Z"], mj_bodies_xpos[unreal_actor_name + ":StaticMeshComponent0"])),
                    "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], unreal_rpy_from_mujoco_quaternion(mj_bodies_xquat[unreal_actor_name + ":StaticMeshComponent0"]))),
                    "bSweep": False,
                    "bTeleport": True})

            # update articulated actors
            for unreal_actor_name, unreal_actor in unreal_articulated_actors.items():
                # update root component transform
                mujoco_actor_root_component_name = unreal_actor_name + ":DefaultSceneRoot"
                if mujoco_actor_root_component_name in mj_bodies_xpos:
                    args = {
                        "NewLocation": dict(zip(["X", "Y", "Z"], mj_bodies_xpos[mujoco_actor_root_component_name])),
                        "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], unreal_rpy_from_mujoco_quaternion(mj_bodies_xquat[mujoco_actor_root_component_name]))),
                        "bSweep": False,
                        "bTeleport": True}
                    instance.unreal_service.call_function(unreal_actor, set_actor_location_and_rotation_func, args)

                # update component transform
                for unreal_component_name, unreal_component in unreal_articulated_actor_components[unreal_actor_name].items():
                    mujoco_component_name = unreal_actor_name + ":DefaultSceneRoot." + unreal_component_name
                    if mujoco_component_name in mj_bodies_xpos:
                        instance.unreal_service.call_function(unreal_component, set_component_relative_location_and_rotation_func, {
                            "NewLocation": dict(zip(["X", "Y", "Z"], mj_bodies_xpos[mujoco_component_name])),
                            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], unreal_rpy_from_mujoco_quaternion(mj_bodies_xquat[mujoco_component_name]))),
                            "bSweep": False,
                            "bTeleport": True})

        with instance.end_frame():
            pass

    mj_viewer.close()
    instance.close()

    spear.log("Done.")
