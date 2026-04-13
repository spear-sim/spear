#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# Demonstrates the latency difference between single-buffered, double-buffered, and triple-buffered readback.
# Each frame spawns an object at a different horizontal position, so the number of visible objects
# unambiguously identifies which frame's pixels are being returned.

import cv2
import os
import spear


def initialize_camera(game, bp_camera_sensor_uclass, viewport_info, buffering_mode):

    bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
    component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")
    game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=component, viewport_info=viewport_info, widths=512, heights=512)

    if buffering_mode is not None:
        component.BufferingMode = buffering_mode

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
        viewport_info = game.rendering_service.get_current_viewport_info()

        sb_camera, sb_component = initialize_camera(game=game, bp_camera_sensor_uclass=bp_camera_sensor_uclass, viewport_info=viewport_info, buffering_mode="SingleBuffered")
        db_camera, db_component = initialize_camera(game=game, bp_camera_sensor_uclass=bp_camera_sensor_uclass, viewport_info=viewport_info, buffering_mode="DoubleBuffered")
        tb_camera, tb_component = initialize_camera(game=game, bp_camera_sensor_uclass=bp_camera_sensor_uclass, viewport_info=viewport_info, buffering_mode="TripleBuffered")

    with instance.end_frame(single_step=True):
        pass

    instance.step(num_frames=2) # two flush frames are occasionally needed on Windows

    spawn_y_positions = [200.0, 260.0, 320.0]

    #
    # single-buffered: spawn one object per frame, read pixels each frame (0 frames of latency)
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
    # double-buffered: spawn one object per frame, enqueue_copy + read pixels each frame (1 frame of latency)
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

    spear.log("Executing double-buffered frame: no spawn (trailing frame 1)")

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

    #
    # triple-buffered: spawn one object per frame, enqueue_copy + read pixels each frame (2 frames of latency)
    #

    # priming frame 1
    with instance.begin_frame():
        tb_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        pass

    # priming frame 2
    with instance.begin_frame():
        tb_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        pass

    tb_objects = []

    for i, y in enumerate(spawn_y_positions):
        spear.log(f"Executing triple-buffered frame: spawn object {i + 1}")

        with instance.begin_frame():
            obj = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": 40.0, "Y": y, "Z": 50.0})
            obj.SetActorScale3D(NewScale3D={"X": 2.0, "Y": 2.0, "Z": 2.0})
            tb_objects.append(obj)
            tb_component.enqueue_copy()
        with instance.end_frame(single_step=True):
            data_bundle = tb_component.read_pixels()

        cv2.imshow(f"Triple-buffered frame: expecting {max(i - 1, 0)} object(s)", data_bundle["arrays"]["data"])
        cv2.waitKey(0)

    spear.log("Executing triple-buffered frame: no spawn (trailing frame 1)")

    with instance.begin_frame():
        tb_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        data_bundle = tb_component.read_pixels()

    cv2.imshow(f"Triple-buffered frame: expecting {len(spawn_y_positions) - 1} object(s)", data_bundle["arrays"]["data"])
    cv2.waitKey(0)

    spear.log("Executing triple-buffered frame: no spawn (trailing frame 2)")

    with instance.begin_frame():
        tb_component.enqueue_copy()
    with instance.end_frame(single_step=True):
        data_bundle = tb_component.read_pixels()

    cv2.imshow(f"Triple-buffered frame: expecting {len(spawn_y_positions)} object(s)", data_bundle["arrays"]["data"])
    cv2.waitKey(0)

    with instance.begin_frame():
        for obj in tb_objects:
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
        terminate_camera(game=game, bp_camera_sensor=tb_camera, component=tb_component)

    instance.step() # needed after the last call to instance.end_frame(single_step=True)

    spear.log("Done.")
