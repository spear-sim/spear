#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# Demonstrates the one-frame latency difference between single-buffered and double-buffered readback.
# Each frame spawns an object at a different horizontal position, so the number of visible objects
# unambiguously identifies which frame's pixels are being returned.

import cv2
import math
import os
import spear


def initialize_camera(game, bp_camera_sensor_uclass, location, rotation, fov_adjusted_degrees, post_process_volume_settings, double_buffered):

    bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
    component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

    bp_camera_sensor.K2_SetActorLocation(NewLocation=location)
    bp_camera_sensor.K2_SetActorRotation(NewRotation=rotation)

    component.Width = 512
    component.Height = 512
    component.FOVAngle = fov_adjusted_degrees
    if post_process_volume_settings is not None:
        component.PostProcessSettings = post_process_volume_settings

    if double_buffered:
        component.bUseDoubleBufferedReadback = True

    component.Initialize()
    component.initialize_sp_funcs()

    return bp_camera_sensor, component


def terminate_camera(game, bp_camera_sensor, component):
    component.terminate_sp_funcs()
    component.Terminate()
    game.unreal_service.destroy_actor(actor=bp_camera_sensor)


if __name__ == "__main__":

    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")

        engine = game.engine_globals_service.get_engine()
        game_viewport_client = engine.GameViewport.get()

        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
        player_camera_manager = player_controller.PlayerCameraManager.get()
        view_target_pov = player_camera_manager.ViewTarget.POV.get()

        post_process_volume_settings = None
        post_process_volumes = game.unreal_service.find_actors_by_class(uclass="APostProcessVolume")
        if len(post_process_volumes) == 1:
            post_process_volume_settings = post_process_volumes[0].Settings.get()

        sp_game_viewport = game.get_unreal_object(uclass="USpGameViewportClient")
        return_values = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)

        viewport_size_x = return_values["ViewportSize"]["x"]
        viewport_size_y = return_values["ViewportSize"]["y"]
        viewport_aspect_ratio = viewport_size_x / viewport_size_y
        fov = view_target_pov["fOV"] * math.pi / 180.0
        half_fov = fov / 2.0
        half_fov_adjusted = math.atan(math.tan(half_fov) * viewport_aspect_ratio / view_target_pov["aspectRatio"])
        fov_adjusted_degrees = half_fov_adjusted * 2.0 * 180.0 / math.pi

        location = view_target_pov["location"]
        rotation = view_target_pov["rotation"]

        sb_camera, sb_component = initialize_camera(
            game=game,
            bp_camera_sensor_uclass=bp_camera_sensor_uclass,
            location=location,
            rotation=rotation,
            fov_adjusted_degrees=fov_adjusted_degrees,
            post_process_volume_settings=post_process_volume_settings,
            double_buffered=False)

        db_camera, db_component = initialize_camera(
            game=game,
            bp_camera_sensor_uclass=bp_camera_sensor_uclass,
            location=location,
            rotation=rotation,
            fov_adjusted_degrees=fov_adjusted_degrees,
            post_process_volume_settings=post_process_volume_settings,
            double_buffered=True)

    with instance.end_frame(single_step=True):
        pass

    for i in range(1):
        instance.flush()

    spawn_y_positions = [200.0, 260.0, 320.0]

    #
    # SINGLE-BUFFERED: spawn one object per frame, read pixels each frame
    #

    sb_objects = []

    for i, y in enumerate(spawn_y_positions):
        spear.log(f"Executing single-buffered frame: spawn object {i + 1}")

        with instance.begin_frame():
            obj = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": 40.0, "Y": y, "Z": 50.0})
            obj.SetActorScale3D(NewScale3D={"X": 2.0, "Y": 2.0, "Z": 2.0})
            sb_objects.append(obj)
        with instance.end_frame(single_step=True):
            data_bundle = sb_component.read_pixels()

        cv2.imshow(f"Single-buffered frame: expecting {i + 1} object(s)", data_bundle["arrays"]["data"])
        cv2.waitKey(0)

    with instance.begin_frame():
        for obj in sb_objects:
            game.unreal_service.destroy_actor(actor=obj)
    with instance.end_frame(single_step=True):
        pass

    #
    # DOUBLE-BUFFERED: spawn one object per frame, read pixels each frame (one frame behind)
    #

    # priming frame
    with instance.begin_frame():
        db_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        pass

    db_objects = []

    for i, y in enumerate(spawn_y_positions):
        spear.log(f"Executing double-buffered frame: spawn object {i + 1}")

        with instance.begin_frame():
            obj = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": 40.0, "Y": y, "Z": 50.0})
            obj.SetActorScale3D(NewScale3D={"X": 2.0, "Y": 2.0, "Z": 2.0})
            db_objects.append(obj)
            db_component.enqueue_copy()
        with instance.end_frame(single_step=True):
            data_bundle = db_component.read_pixels()

        cv2.imshow(f"Double-buffered frame: expecting {i} object(s)", data_bundle["arrays"]["data"])
        cv2.waitKey(0)

    spear.log("Executing double-buffered frame: no spawn")

    with instance.begin_frame():
        db_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        data_bundle = db_component.read_pixels()

    cv2.imshow(f"Double-buffered frame: expecting {len(spawn_y_positions)} object(s)", data_bundle["arrays"]["data"])
    cv2.waitKey(0)

    with instance.begin_frame():
        for obj in db_objects:
            game.unreal_service.destroy_actor(actor=obj)
    with instance.end_frame(single_step=True):
        pass

    cv2.destroyAllWindows()

    # terminate
    with instance.begin_frame():
        pass
    with instance.end_frame(single_step=True):
        terminate_camera(game=game, bp_camera_sensor=sb_camera, component=sb_component)
        terminate_camera(game=game, bp_camera_sensor=db_camera, component=db_component)

    instance.flush() # needed after the last call to instance.end_frame(single_step=True)

    spear.log("Done.")
