#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import math
import mujoco
import mujoco.viewer
import numpy as np
import os
import scipy
import shutil
import spear

width = 1280
height = 720


parser = argparse.ArgumentParser()
parser.add_argument("--mjcf-file", required=True)
parser.add_argument("--save-images", action="store_true")
parser.add_argument("--visual-parity-with-unreal", action="store_true")
args = parser.parse_args()

assert os.path.exists(args.mjcf_file)

num_ue_steps_per_saved_image = 10
num_mj_steps_per_ue_step = 10
name_prefix = "Meshes/05_chair"

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr",
        "visualize_func": lambda data : data[:,:,[0,1,2]] # native BGR (drop alpha)
    }
]

def normalize(vector):
    return vector / np.linalg.norm(vector)

def to_spear_vector_from_mujoco_array(mujoco_array):
    numpy_array = mujoco_array
    if args.visual_parity_with_unreal:
        numpy_array = (np.diag([1,-1,1])*np.matrix(numpy_array).T).A1
    return spear.math.to_spear_vector_from_numpy_array(numpy_array=numpy_array)

def to_spear_rotator_from_mujoco_matrix(mujoco_matrix):
    numpy_matrix = mujoco_matrix
    if args.visual_parity_with_unreal:
        numpy_matrix = np.diag([1,-1,1])*np.matrix(numpy_matrix)*np.diag([1,-1,1])
    return spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=numpy_matrix)

def to_spear_rotator_from_mujoco_quat(mujoco_quat):
    # MuJoCo assumes quaternions are stored in scalar-first (wxyz) order, but scipy.spatial.transform.Rotation
    # assumes scalar-last (xyzw) order
    numpy_quat = mujoco_quat[[1,2,3,0]]
    mujoco_matrix = scipy.spatial.transform.Rotation.from_quat(numpy_quat).as_matrix()
    return to_spear_rotator_from_mujoco_matrix(mujoco_matrix=mujoco_matrix)

def to_mujoco_array_from_spear_vector(spear_vector):
    numpy_array = spear.math.to_numpy_array_from_spear_vector(spear_vector=spear_vector)
    if args.visual_parity_with_unreal:
        numpy_array = (np.diag([1,-1,1])*np.matrix(numpy_array).T).A1
    return numpy_array

def to_mujoco_matrix_from_spear_rotator(spear_rotator):
    numpy_matrix = spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=spear_rotator)
    if args.visual_parity_with_unreal:
        numpy_matrix = (np.diag([1,-1,1])*np.matrix(numpy_matrix)*np.diag([1,-1,1])).A
    return numpy_matrix


if __name__ == "__main__":

    # create output dirs
    images_dir = spear.path.realpath(os.path.dirname(__file__), "images")
    if args.save_images:
        if os.path.exists(images_dir):
            spear.log("Directory exists, removing: ", images_dir)
            shutil.rmtree(images_dir, ignore_errors=True)
        os.makedirs(images_dir, exist_ok=True)
        for component_desc in component_descs:
            os.makedirs(spear.path.realpath(images_dir, component_desc["name"]), exist_ok=True)
        os.makedirs(spear.path.realpath(images_dir, "mujoco"), exist_ok=True)

    # create SPEAR instance
    config = spear.get_config(user_config_files=[spear.path.realpath(os.path.dirname(__file__), "user_config.yaml")])
    spear.configure_system(config=config)
    sp_instance = spear.Instance(config=config)
    sp_game = sp_instance.get_game()

    # initialize actors and components
    with sp_instance.begin_frame():

        ue_actors = sp_game.unreal_service.find_actors_as_dict(include_unreal_id=True)
        ue_actors = { ue_actor_name: ue_actor for ue_actor_name, ue_actor in ue_actors.items() if ue_actor_name.startswith(name_prefix) }

        # get player controller
        player_controller = sp_game.cdo.UGameplayStatics.GetPlayerController(PlayerIndex=0)

        # get pawn
        pawn = player_controller.K2_GetPawn()

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = sp_game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = sp_game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # get camera components

        final_tone_curve_hdr_component = None

        for component_desc in component_descs:
            component_desc["component"] = sp_game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=component_desc["long_name"], uclass="USpSceneCaptureComponent2D")

            # get final_tone_curve_hdr_component
            if component_desc["name"] == "final_tone_curve_hdr":
                final_tone_curve_hdr_component = component_desc["component"]

        assert final_tone_curve_hdr_component is not None

        # configure components to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = sp_game.viewport_service.get_current_viewport_desc()
        components = [ desc["component"] for desc in component_descs ]
        sp_game.viewport_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_desc=viewport_desc, widths=width, heights=height)

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with sp_instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(spear.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # get MuJoCo bodies
    mj_bodies = { mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if mj_model.body(mj_body).name.startswith(name_prefix) }

    # need to set mj_model.vis properties before launching the viewer
    fov_y_degrees = viewport_desc["fov_y_degrees"]
    mj_model.vis.global_.fovy = fov_y_degrees
    mj_model.vis.headlight.ambient = [0.4, 0.4, 0.4]

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data, show_right_ui=False)

    # initialize MuJoCo camera (not needed when launching the viewer through the command-line, but needed when using launch_passive)

    cam_position = to_mujoco_array_from_spear_vector(spear_vector=viewport_desc["camera_location"])
    cam_rotation_matrix = to_mujoco_matrix_from_spear_rotator(spear_rotator=viewport_desc["camera_rotation"])

    # convert camera position and rotation matrix to MuJoCo's look-at camera parameterization

    cam_rotation_x_axis = cam_rotation_matrix[:,0]
    lookat_distance = 5.0*100.0 # 5 meters * 100 Unreal units per meter
    lookat_dir = cam_rotation_x_axis # Unreal's camera conventions are: x-axis is forward, y-axis is right, z-axis is up
    lookat_pos = cam_position + lookat_dir*lookat_distance
    lookat_azimuth_degrees = math.atan2(lookat_dir[1], lookat_dir[0])*180.0/math.pi
    lookat_elevation_degrees = math.atan2(lookat_dir[2], math.sqrt(lookat_dir[0]*lookat_dir[0] + lookat_dir[1]*lookat_dir[1]))*180.0/math.pi

    mj_viewer.cam.distance = lookat_distance
    mj_viewer.cam.lookat = lookat_pos
    mj_viewer.cam.azimuth = lookat_azimuth_degrees
    mj_viewer.cam.elevation = lookat_elevation_degrees

    # initialize MuJoCo viewer options
    mj_viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION

    # update MuJoCo viewer state
    mj_viewer.sync()

    # initialize MuJoCo screenshot renderer
    if args.save_images:
        mj_renderer = mujoco.Renderer(model=mj_model, height=height, width=width)

    # initialize counters
    ue_step_index = 0
    frame_index = 0

    while mj_viewer.is_running():

        # perform multiple MuJoCo simulation steps per Unreal frame
        for _ in range(num_mj_steps_per_ue_step):
            mujoco.mj_step(mj_model, mj_data)
        mj_viewer.sync()

        # get updated poses from MuJoCo
        mj_bodies_xpos = { mj_body_name: mj_data.body(mj_body).xpos for mj_body_name, mj_body in mj_bodies.items() }
        mj_bodies_xquat = { mj_body_name: mj_data.body(mj_body).xquat for mj_body_name, mj_body in mj_bodies.items() }

        # convert MuJoCo's look-at camera parameterization to a position and rotation matrix

        lookat_pos = np.array(mj_viewer.cam.lookat)
        lookat_distance = mj_viewer.cam.distance
        lookat_azimuth_degrees = mj_viewer.cam.azimuth
        lookat_elevation_degrees = mj_viewer.cam.elevation

        lookat_offset = np.array([
            lookat_distance*math.cos(lookat_azimuth_degrees*math.pi/180.0)*math.cos(lookat_elevation_degrees*math.pi/180.0),
            lookat_distance*math.sin(lookat_azimuth_degrees*math.pi/180.0)*math.cos(lookat_elevation_degrees*math.pi/180.0),
            lookat_distance*math.sin(lookat_elevation_degrees*math.pi/180.0)])

        cam_position = lookat_pos - lookat_offset
        lookat_dir = lookat_pos - cam_position
        up_world = np.array([0.0, 0.0, 1.0])
        cam_rotation_x_axis = normalize(vector=lookat_dir)
        cam_rotation_y_axis = normalize(vector=np.cross(up_world, cam_rotation_x_axis))
        cam_rotation_z_axis = normalize(vector=np.cross(cam_rotation_x_axis, cam_rotation_y_axis))
        cam_rotation_matrix = np.column_stack([cam_rotation_x_axis, cam_rotation_y_axis, cam_rotation_z_axis])

        cam_location = to_spear_vector_from_mujoco_array(mujoco_array=cam_position)
        cam_rotation = to_spear_rotator_from_mujoco_matrix(mujoco_matrix=cam_rotation_matrix)

        with sp_instance.begin_frame():

            # update SPEAR camera pose
            pawn.send_async.K2_SetActorLocation(NewLocation=cam_location)
            player_controller.send_async.SetControlRotation(NewRotation=cam_rotation)
            bp_camera_sensor.send_async.K2_SetActorLocationAndRotation(NewLocation=cam_location, NewRotation=cam_rotation)

            # update SPEAR object poses
            for ue_actor_name, ue_actor in ue_actors.items():
                ue_actor.send_async.K2_SetActorLocationAndRotation(
                    NewLocation=to_spear_vector_from_mujoco_array(mujoco_array=mj_bodies_xpos[f"{ue_actor_name}:StaticMeshComponent0"]),
                    NewRotation=to_spear_rotator_from_mujoco_quat(mujoco_quat=mj_bodies_xquat[f"{ue_actor_name}:StaticMeshComponent0"]),
                    bSweep=False,
                    bTeleport=True)

        with sp_instance.end_frame():

            # read pixels from camera sensor
            if args.save_images and ue_step_index % num_ue_steps_per_saved_image == 0:
                for component_desc in component_descs:
                    data_bundle = component_desc["component"].read_pixels()
                    component_desc["data"] = data_bundle["arrays"]["data"]

        if args.save_images and ue_step_index % num_ue_steps_per_saved_image == 0:

            # save UE images
            spear.image.save_images(component_descs=component_descs, images_dir=images_dir, frame_index=frame_index)

            # save MuJoCo images
            mj_renderer.update_scene(data=mj_data, camera=mj_viewer.cam)
            mj_render = mj_renderer.render()
            image_file = spear.path.realpath(images_dir, "mujoco", f"{frame_index:04d}.png")
            image = mj_render[:,:,[2,1,0]] # MuJoCo renders RGB -> BGR for cv2
            spear.image.save_image(image=image, image_file=image_file)

            frame_index = frame_index + 1

        ue_step_index = ue_step_index + 1

    # terminate actors and components
    with sp_instance.begin_frame():
        pass
    with sp_instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        sp_game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    # close the MuJoCo renderer and viewer, and the SPEAR instance
    if args.save_images:
        mj_renderer.close()
    mj_viewer.close()
    sp_instance.close()

    spear.log("Done.")
