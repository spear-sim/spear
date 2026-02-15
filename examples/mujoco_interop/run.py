#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import math
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import os
import scipy
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--mjcf-file", required=True)
parser.add_argument("--save-images", action="store_true")
parser.add_argument("--visual-parity-with-unreal", action="store_true")
args = parser.parse_args()

num_ue_steps_per_frame = 10
num_mj_steps_per_ue_step = 10
name_prefix = "Meshes/05_chair"

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    }
]

# save an image for each component using the component's visualizer function
def save_images(images_dir, frame_index):
    assert args.save_images
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, component_desc["name"], f"{frame_index:04d}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)

def normalize(vector):
    return vector / np.linalg.norm(vector)

def to_unreal_vector_from_mujoco_vector(mj_vector):
    ue_vector = mj_vector
    if args.visual_parity_with_unreal:
        ue_vector = (np.diag([1,-1,1])*np.matrix(ue_vector).T).A1
    return {"X": ue_vector[0], "Y": ue_vector[1], "Z": ue_vector[2]}

def to_unreal_rotator_from_mujoco_quaternion(mj_quaternion):

    # MuJoCo assumes quaternions are stored in scalar-first (wxyz) order, but scipy.spatial.transform.Rotation assumes scalar-last (xyzw) order
    scipy_quaternion = mj_quaternion[[1,2,3,0]]
    scipy_rotation_matrix = scipy.spatial.transform.Rotation.from_quat(scipy_quaternion).as_matrix()

    if args.visual_parity_with_unreal:
        scipy_rotation_matrix = np.diag([1,-1,1])*np.matrix(scipy_rotation_matrix)*np.diag([1,-1,1])

    # Unreal and scipy.spatial.transform.Rotation have different Euler angle conventions, see python/spear/utils/pipeline_utils.py for details
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_matrix(scipy_rotation_matrix).as_euler("xyz")

    ue_pitch = np.rad2deg(-scipy_pitch)
    ue_yaw   = np.rad2deg(scipy_yaw)
    ue_roll  = np.rad2deg(-scipy_roll)

    return {"Roll": ue_roll, "Pitch": ue_pitch, "Yaw": ue_yaw}


if __name__ == "__main__":

    # create output dirs
    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
    if args.save_images:
        if os.path.exists(images_dir):
            spear.log("Directory exists, removing: ", images_dir)
            shutil.rmtree(images_dir, ignore_errors=True)
        os.makedirs(images_dir, exist_ok=True)
        for component_desc in component_descs:
            os.makedirs(os.path.realpath(os.path.join(images_dir, component_desc["name"])), exist_ok=True)
        os.makedirs(os.path.realpath(os.path.join(images_dir, "mujoco")), exist_ok=True)

    # create SPEAR instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    sp_instance = spear.Instance(config=config)
    sp_game = sp_instance.get_game()

    # initialize actors and components
    with sp_instance.begin_frame():

        ue_actors = sp_game.unreal_service.find_actors_as_dict()
        ue_actors = { ue_actor_name: ue_actor for ue_actor_name, ue_actor in ue_actors.items() if ue_actor_name.startswith(name_prefix) }

        # get UGameplayStatics
        gameplay_statics = sp_game.get_unreal_object(uclass="UGameplayStatics")

        # get player controller
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)

        # get character
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
        
        player_camera_manager = player_controller.PlayerCameraManager.get()
        view_target_pov = player_camera_manager.ViewTarget.POV.get()

        viewport_size_x = 1280
        viewport_size_y = 720

        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y
        fov = view_target_pov["fOV"]*math.pi/180.0
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"]) # this adjustment is necessary to compute an FOV value that matches the game viewport
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

        for component_desc in component_descs:
            component_desc["component"].Width = viewport_size_x*component_desc["spatial_supersampling_factor"]
            component_desc["component"].Height = viewport_size_y*component_desc["spatial_supersampling_factor"]
            component_desc["component"].FOVAngle = fov_adjusted_degrees

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with sp_instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # get MuJoCo bodies
    mj_bodies = { mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if mj_model.body(mj_body).name.startswith(name_prefix) }

    # need to set mj_model.vis properties before launching the viewer
    fov_y_degrees = 2.0*math.atan(math.tan(fov_adjusted/2.0)/viewport_aspect_ratio)*180.0/math.pi # fov_x -> fov_y -> fov_y_degrees
    mj_model.vis.global_.fovy = fov_y_degrees
    mj_model.vis.headlight.ambient = [0.4, 0.4, 0.4]

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    # initialize MuJoCo camera (not needed when launching the viewer through the command-line, but needed when using launch_passive)

    cam_location = view_target_pov["location"]
    cam_rotator = view_target_pov["rotation"]

    cam_position = spear.to_numpy_array_from_vector(vector=cam_location)
    cam_rotation_matrix = spear.to_numpy_matrix_from_rotator(rotator=cam_rotator)

    if args.visual_parity_with_unreal:
        cam_position = (np.diag([1,-1,1])*np.matrix(cam_position).T).A1
        cam_rotation_matrix = (np.diag([1,-1,1])*np.matrix(cam_rotation_matrix)*np.diag([1,-1,1])).A

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
    image_width = viewport_size_x
    image_height = viewport_size_y
    mj_renderer = mujoco.Renderer(model=mj_model, height=image_height, width=image_width)

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
        cam_rotation_x_axis = normalize(lookat_dir)
        cam_rotation_y_axis = normalize(np.cross(up_world, cam_rotation_x_axis))
        cam_rotation_z_axis = normalize(np.cross(cam_rotation_x_axis, cam_rotation_y_axis))
        cam_rotation_matrix = np.column_stack([cam_rotation_x_axis, cam_rotation_y_axis, cam_rotation_z_axis])

        if args.visual_parity_with_unreal:
            cam_position = (np.diag([1,-1,1])*np.matrix(cam_position).T).A1
            cam_rotation_matrix = (np.diag([1,-1,1])*np.matrix(cam_rotation_matrix)*np.diag([1,-1,1])).A

        ue_actor_futures = {}
        with sp_instance.begin_frame():

            # update SPEAR camera pose
            pawn_future = pawn.call_async.K2_SetActorLocation(NewLocation=spear.to_vector_from_numpy_array(array=cam_position))
            player_controller_future = player_controller.call_async.SetControlRotation(NewRotation=spear.to_rotator_from_numpy_matrix(matrix=cam_rotation_matrix))

            bp_camera_sensor.K2_SetActorLocation(NewLocation=spear.to_vector_from_numpy_array(array=cam_position))
            bp_camera_sensor.K2_SetActorRotation(NewRotation=spear.to_rotator_from_numpy_matrix(matrix=cam_rotation_matrix))

            # update SPEAR object poses
            for ue_actor_name, ue_actor in ue_actors.items():
                ue_actor_futures[ue_actor_name] = ue_actor.call_async.K2_SetActorLocationAndRotation(
                    NewLocation=to_unreal_vector_from_mujoco_vector(mj_bodies_xpos[f"{ue_actor_name}:StaticMeshComponent0"]),
                    NewRotation=to_unreal_rotator_from_mujoco_quaternion(mj_bodies_xquat[f"{ue_actor_name}:StaticMeshComponent0"]),
                    bSweep=False,
                    bTeleport=True)

        with sp_instance.end_frame():

            # clean up futures
            pawn_future.get()
            player_controller_future.get()
            for ue_actor_name in ue_actors.keys():
                ue_actor_futures[ue_actor_name].get()

            # read pixels from camera sensor
            if args.save_images:
                if ue_step_index % num_ue_steps_per_frame == 0:
                    for component_desc in component_descs:
                        data_bundle = component_desc["component"].read_pixels()
                        component_desc["data"] = data_bundle["arrays"]["data"]

        # save images
        if args.save_images:
            if ue_step_index % num_ue_steps_per_frame == 0:
                save_images(images_dir=images_dir, frame_index=frame_index)
                frame_index = frame_index + 1

        if args.save_images:
            if ue_step_index % num_ue_steps_per_frame == 0:
                mj_renderer.update_scene(data=mj_data, camera=mj_viewer.cam)
                mj_render = mj_renderer.render()
                image_file = os.path.realpath(os.path.join(images_dir, "mujoco", f"{frame_index:04d}.png"))
                image = mj_render
                spear.log("Saving image: ", image_file)
                plt.imsave(image_file, image)

        ue_step_index = ue_step_index + 1

    # terminate actors and components
    with sp_instance.begin_frame():
        pass
    with sp_instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        sp_game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    # close MuJoCo viewer and SPEAR instance
    mj_viewer.close()
    sp_instance.close()

    spear.log("Done.")
