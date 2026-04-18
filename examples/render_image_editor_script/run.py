#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import matplotlib.pyplot as plt
import os
import spear
import unreal # not needed but useful to demonstrate that we're running inside the editor


parser = argparse.ArgumentParser()
parser.add_argument("--teaser", action="store_true")
args = parser.parse_args()

if args.teaser:
    width = 1920
    height = 1080
else:
    width = None
    height = None

file = __file__


@spear.editor.script
def script():

    instance = spear.Instance()
    game = yield from instance.get_game_in_editor_script()

    # initialize actors and components
    with instance.begin_frame():
        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc, widths=width, heights=height)

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()
    yield

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames
    yield

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame or two can fix occasional render-to-texture initialization issues
    yield from instance.flush_in_editor_script(num_frames=2)

    # get rendered frame
    with instance.begin_frame():
        pass
    yield

    with instance.end_frame():
        data_bundle = final_tone_curve_hdr_component.read_pixels()
    yield

    # show rendered frame using spear.editor.imshow() since cv2 and matplotlib cause problems in the editor
    spear.editor.imshow(name="final_tone_curve_hdr", image=data_bundle["arrays"]["data"])

    # save image
    image = data_bundle["arrays"]["data"][:,:,[2,1,0]]
    image_file = os.path.realpath(os.path.join(os.path.dirname(file), "image.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    yield

    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)
    yield

    spear.log("Done.")


if __name__ == "__main__":
    script()
