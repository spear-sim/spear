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
import time


parser = argparse.ArgumentParser()
parser.add_argument("--benchmark", action="store_true")
parser.add_argument("--double-buffered-readback", action="store_true")
parser.add_argument("--no-shared-memory", action="store_true")
parser.add_argument("--print-frame-time-every-frame", action="store_true")
parser.add_argument("--print-readback-spin-wait-info", action="store_true")
parser.add_argument("--read-pixels-every-frame", action="store_true")
parser.add_argument("--triple-buffered-readback", action="store_true")
args = parser.parse_args()

if args.benchmark:
    width = 1920
    height = 1080
else:
    width = None
    height = None


if __name__ == "__main__":

    assert args.double_buffered_readback + args.triple_buffered_readback <= 1
    shared_memory = not args.no_shared_memory
    buffered_readback = args.double_buffered_readback or args.triple_buffered_readback
    if args.triple_buffered_readback:
        num_priming_frames = 2
    elif args.double_buffered_readback:
        num_priming_frames = 1
    else:
        num_priming_frames = 0

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc, widths=width, heights=height)

        if not shared_memory:
            final_tone_curve_hdr_component.bUseSharedMemory = False

        if args.double_buffered_readback:
            final_tone_curve_hdr_component.BufferingMode = "DoubleBuffered"
        elif args.triple_buffered_readback:
            final_tone_curve_hdr_component.BufferingMode = "TripleBuffered"

        if args.print_readback_spin_wait_info:
            final_tone_curve_hdr_component.bPrintReadbackSpinWaitInfo = True

        # update state for measuring "standalone" and "standalone + extra work" frame rates
        if args.print_frame_time_every_frame:
            final_tone_curve_hdr_component.bPrintFrameTimeEveryFrame = True
        if args.read_pixels_every_frame:
            final_tone_curve_hdr_component.bReadPixelsEveryFrame = True

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame or two can fix occasional render-to-texture initialization issues
    instance.step(num_frames=2)

    # get rendered frame
    if buffered_readback:

        # priming frames
        for i in range(num_priming_frames):
            with instance.begin_frame():
                future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
            with instance.end_frame():
                future_enqueue_copy.get()

        # steady state frame
        with instance.begin_frame():
            future_read_pixels = final_tone_curve_hdr_component.call_async.read_pixels()
            future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
        with instance.end_frame():
            data_bundle = future_read_pixels.get()
            future_enqueue_copy.get()

    else:
        with instance.begin_frame():
            future = final_tone_curve_hdr_component.call_async.read_pixels()
        with instance.end_frame():
            data_bundle = future.get()

    # optional benchmarking
    if args.benchmark:

        # update state for measuring "standalone" and "standalone + extra work" frame rates

        if args.print_frame_time_every_frame and not args.read_pixels_every_frame:
            with instance.begin_frame():
                # if we're not reading pixels every frame, turn rendering off so we're only measuring "standalone" performance
                final_tone_curve_hdr_component.SetVisibility(bNewVisibility=False)
            with instance.end_frame():
                pass

        if args.print_frame_time_every_frame or args.read_pixels_every_frame:
            time.sleep(30.0) # sleep for a few seconds so we can benchmark "standalone" and "standalone + extra work" frame rates

            with instance.begin_frame():
                final_tone_curve_hdr_component.terminate_sp_funcs()
                final_tone_curve_hdr_component.Terminate()

                # now that we're finished measuring "standalone" and "standalone + extra work", turn this extra functionality off
                if args.print_frame_time_every_frame:
                    final_tone_curve_hdr_component.bPrintFrameTimeEveryFrame = False
                if args.read_pixels_every_frame:
                    final_tone_curve_hdr_component.bReadPixelsEveryFrame = False

                final_tone_curve_hdr_component.Initialize()
                final_tone_curve_hdr_component.initialize_sp_funcs()
            with instance.end_frame():
                pass

        # instance._client.get_timeout()
        num_steps = 100000
        start_time_seconds = time.time()
        for i in range(num_steps):
            instance._client.get_timeout()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average time for instance._client.get_timeout(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.engine_globals_service.get_current_process_id()
        num_steps = 1000
        start_time_seconds = time.time()
        for i in range(num_steps):
            instance.engine_globals_service.get_current_process_id()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average time for instance.engine_globals_service.get_current_process_id(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # empty with instance.begin_frame() / with instance.end_frame()
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                pass
            with instance.end_frame():
                pass
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for empty with instance.begin_frame() / with instance.end_frame(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # game.unreal_service.get_default_object(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                pass
            with instance.end_frame():
                return_value = game.unreal_service.get_default_object(uclass="AActor", as_handle=True)
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for game.unreal_service.get_default_object(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # game.unreal_service.call_async.get_default_object(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                future = game.unreal_service.call_async.get_default_object(uclass="AActor", as_handle=True)
            with instance.end_frame():
                return_value = future.get()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for game.unreal_service.call_async.get_default_object(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_function(...)
        num_steps = 100
        if buffered_readback:
            for i in range(num_priming_frames):
                with instance.begin_frame():
                    final_tone_curve_hdr_component.enqueue_copy()
                with instance.end_frame():
                    pass
        start_time_seconds = time.time()
        if buffered_readback:
            for i in range(num_steps):
                with instance.begin_frame():
                    data_bundle = final_tone_curve_hdr_component.read_pixels()
                    final_tone_curve_hdr_component.enqueue_copy()
                with instance.end_frame():
                    pass
        else:
            for i in range(num_steps):
                with instance.begin_frame():
                    data_bundle = final_tone_curve_hdr_component.read_pixels()
                with instance.end_frame():
                    pass
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_function(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_function(...) (single-step)
        num_steps = 100
        if buffered_readback:
            for i in range(num_priming_frames):
                with instance.begin_frame():
                    final_tone_curve_hdr_component.enqueue_copy()
                with instance.end_frame(single_step=True):
                    pass
        start_time_seconds = time.time()
        if buffered_readback:
            for i in range(num_steps):
                with instance.begin_frame():
                    data_bundle = final_tone_curve_hdr_component.read_pixels()
                    final_tone_curve_hdr_component.enqueue_copy()
                with instance.end_frame(single_step=True):
                    pass
        else:
            for i in range(num_steps):
                with instance.begin_frame():
                    data_bundle = final_tone_curve_hdr_component.read_pixels()
                with instance.end_frame(single_step=True):
                    pass
        end_time_seconds = time.time()
        instance.step() # needed after the last call to instance.end_frame(single_step=True)
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_function(...) (single-step): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_async.call_function(...)
        num_steps = 100
        if buffered_readback:
            for i in range(num_priming_frames):
                with instance.begin_frame():
                    future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
                with instance.end_frame():
                    future_enqueue_copy.get()
        start_time_seconds = time.time()
        if buffered_readback:
            for i in range(num_steps):
                with instance.begin_frame():
                    future_read_pixels = final_tone_curve_hdr_component.call_async.read_pixels()
                    future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
                with instance.end_frame():
                    data_bundle = future_read_pixels.get()
                    future_enqueue_copy.get()
        else:
            for i in range(num_steps):
                with instance.begin_frame():
                    future = final_tone_curve_hdr_component.call_async.read_pixels()
                with instance.end_frame():
                    data_bundle = future.get()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_async.call_function(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_async.call_function(...) (single-step)
        num_steps = 100
        if buffered_readback:
            for i in range(num_priming_frames):
                with instance.begin_frame():
                    future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
                with instance.end_frame(single_step=True):
                    future_enqueue_copy.get()
        start_time_seconds = time.time()
        if buffered_readback:
            for i in range(num_steps):
                with instance.begin_frame():
                    future_read_pixels = final_tone_curve_hdr_component.call_async.read_pixels()
                    future_enqueue_copy = final_tone_curve_hdr_component.call_async.enqueue_copy()
                with instance.end_frame(single_step=True):
                    data_bundle = future_read_pixels.get()
                    future_enqueue_copy.get()
        else:
            for i in range(num_steps):
                with instance.begin_frame():
                    future = final_tone_curve_hdr_component.call_async.read_pixels()
                with instance.end_frame(single_step=True):
                    data_bundle = future.get()
        end_time_seconds = time.time()
        instance.step() # needed after the last call to instance.end_frame(single_step=True)
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_async.call_function(...) (single-step): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

    input("Press any key to continue...")

    # show debug data now that we're outside of instance.end_frame()
    spear.log('data_bundle["arrays"]["data"].flags["ALIGNED"]: ', data_bundle["arrays"]["data"].flags["ALIGNED"])

    # show rendered frame now that we're outside of with instance.end_frame()
    cv2.imshow("final_tone_curve_hdr", data_bundle["arrays"]["data"])
    cv2.waitKey(0)

    # save image
    image = data_bundle["arrays"]["data"][:,:,[2,1,0]]
    image_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "image.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    if args.benchmark:
        instance.close()

    spear.log("Done.")
