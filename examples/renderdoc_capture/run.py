#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import os
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # force high-res textures for captured images
        game.console_service.set(name="r.Streaming.FullyLoadUsedTextures", value=1)

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc)

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame():
        pass

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and inserting an
    # extra frame or two can fix occasional render-to-texture initialization issues (advances a minimum of 3 frames)
    game.async_loading_service.wait_for_engine_idle()

    # set-up the scene capture component to do an isolated RenderDoc GPU capture
    with instance.begin_frame():
        final_tone_curve_hdr_component.bCaptureGpuNextRender = True
    with instance.end_frame():
        data_bundle = final_tone_curve_hdr_component.read_pixels()

    spear.log("RenderDoc capture finished, see <Project>/Saved/RenderDocCaptures/ for results")

    # save image (cv2 writes the native BGR, dropping alpha), so we can confirm the offscreen render is correct
    image_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "image.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, data_bundle["arrays"]["data"][:,:,[0,1,2]])

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Done.")
