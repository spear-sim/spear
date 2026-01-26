#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--skip-save-images", action="store_true")
parser.add_argument("--skip-read-pixels", action="store_true")
args = parser.parse_args()

num_frames = 11
num_frames_for_drone_update = 15
num_frames_for_pcg_update = 60

location_start = np.array([93130.0, -4690.0, 1660.0])
location_end = np.array([87070.0, -3860.0, 1660.0])

rotator_pyr_start = np.array([0.0, -80.0, 0.0])
rotator_pyr_end = np.array([0.0, -80.0, 0.0])

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    }
]

bp_pcg_assembly_descs = \
[
    { "name": "/Game/PCG/Assets/BP_PCG_LargeAssembly.BP_PCG_LargeAssembly_C" },
    { "name": "/Game/PCG/Assets/BP_PCG_SmallAssembly.BP_PCG_SmallAssembly_C" },
    { "name": "/Game/PCG/Graphs/Ditch/PCGDemo_DitchBP.PCGDemo_DitchBP_C" },
    { "name": "/Game/PCG/Graphs/Ground/PCGDemo_GroundBP.PCGDemo_GroundBP_C" }
]

# save an image for each component using the component's visualizer function
def save_images(images_dir, frame_index):
    assert not args.skip_save_images
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, component_desc["name"], f"{frame_index:04d}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)


if __name__ == "__main__":

    # create output dirs
    if not args.skip_read_pixels and not args.skip_save_images:
        images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", "pcg_assembly"))
        if os.path.exists(images_dir):
            spear.log("Directory exists, removing: ", images_dir)
            shutil.rmtree(images_dir, ignore_errors=True)
        os.makedirs(images_dir, exist_ok=True)
        for component_desc in component_descs:
            os.makedirs(os.path.realpath(os.path.join(images_dir, component_desc["name"])), exist_ok=True)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # get player controller
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)

        # get and configure drone
        drone = player_controller.K2_GetPawn()
        drone.K2_SetActorLocation(NewLocation={"X": 90220.0, "Y": -7610.0, "Z": 1940.0})

        # get PCG subsystem and assemblies

        sp_pcg_subsystem = game.get_unreal_object(uclass="USpPCGSubsystem")
        pcg_subsystem = game.unreal_service.get_subsystem(subsystem_provider_class_name="UWorld", subsystem_uclass="UPCGSubsystem")
        assert sp_pcg_subsystem.IsInitialized(PCGSubsystem=pcg_subsystem)

        bp_pcg_large_assembly = None

        for bp_pcg_assembly_desc in bp_pcg_assembly_descs:
            bp_pcg_assembly_desc["uclass"] = game.unreal_service.load_class(uclass="AActor", name=bp_pcg_assembly_desc["name"])
            bp_pcg_assembly_desc["actor"] = game.unreal_service.find_actor_by_class(uclass=bp_pcg_assembly_desc["uclass"])
            bp_pcg_assembly_desc["pcg_components"] = game.unreal_service.get_components_by_class(actor=bp_pcg_assembly_desc["actor"], uclass="UPCGComponent")

            if bp_pcg_assembly_desc["name"] == "/Game/PCG/Assets/BP_PCG_LargeAssembly.BP_PCG_LargeAssembly_C":
                bp_pcg_large_assembly = bp_pcg_assembly_desc["actor"]

        # get and remove instructions
        bp_drone_instructions_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Blueprints/BP_DroneInstructions.BP_DroneInstructions_C")
        bp_drone_instructions = game.unreal_service.find_actors_by_class(uclass=bp_drone_instructions_uclass)
        for bp_drone_instruction in bp_drone_instructions:
            game.unreal_service.destroy_actor(actor=bp_drone_instruction)

        # spawn camera sensor
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # get camera components

        final_tone_curve_hdr_component = None

        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=component_desc["long_name"], uclass="USpSceneCaptureComponent2D")

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
        fov = view_target_pov["fOV"]*math.pi/180.0 # this adjustment is necessary to compute an FOV value that matches the game viewport
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        # make the FOV bigger than the game viewport
        fov_adjusted_degrees = 1.25*fov_adjusted_degrees

        for component_desc in component_descs:
            component_desc["component"].Width = viewport_size_x*component_desc["spatial_supersampling_factor"]
            component_desc["component"].Height = viewport_size_y*component_desc["spatial_supersampling_factor"]
            component_desc["component"].FOVAngle = fov_adjusted_degrees

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    #
    # execute warm-up frames to give Unreal's default auto-exposure settings a chance to settle down, and to
    # give the drone camera actor a chance to rotate, and to give the PCG assemblies a chance to update
    #

    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)

        # set PCG assembly pose
        bp_pcg_large_assembly.K2_SetActorLocation(NewLocation=spear.to_vector_from_numpy_array(array=location_start))
        bp_pcg_large_assembly.K2_SetActorRotation(NewRotation=spear.to_rotator_from_numpy_array(array_pyr=rotator_pyr_start))

        # force PCG updates
        sp_pcg_subsystem.FlushCache(PCGSubsystem=pcg_subsystem)
        for bp_pcg_assembly_desc in bp_pcg_assembly_descs:
            for pcg_component in bp_pcg_assembly_desc["pcg_components"]:
                pcg_component.Generate(bForce=True)

    with instance.end_frame():
        gameplay_statics.SetGamePaused(bPaused=True)

    for _ in range(num_frames_for_drone_update):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)

            # set camera pose
            view_target_pov = player_camera_manager.ViewTarget.POV.get()
            bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
            bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

            # inject input
            instance.enhanced_input_service.inject_input_for_actor(
                actor=drone,
                input_action_name="IA_HoverDrone_Look",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": -1.0, "Y": 0.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # give PCG assemblies a chance to update
    for _ in range(num_frames_for_pcg_update):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
            view_target_pov = player_camera_manager.ViewTarget.POV.get()
            bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
            bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # set camera pose
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
        view_target_pov = player_camera_manager.ViewTarget.POV.get()
        bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])
    with instance.end_frame():
        pass

    for _ in range(175 - num_frames_for_pcg_update - num_frames_for_drone_update):
        instance.flush()

    #
    # initialize frame counter
    #

    frame_index = 0

    #
    # move PCG assembly
    #

    for i, t in enumerate(np.linspace(0.0, 1.0, num_frames)):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)

            location = t*location_end + (1 - t)*location_start
            rotator_pyr = t*rotator_pyr_end + (1 - t)*rotator_pyr_start

            spear.log(f"Set PCG assembly location and rotation: {location}, {rotator_pyr}")

            # set PCG assembly pose
            bp_pcg_large_assembly.K2_SetActorLocation(NewLocation=spear.to_vector_from_numpy_array(array=location))
            bp_pcg_large_assembly.K2_SetActorRotation(NewRotation=spear.to_rotator_from_numpy_array(array_pyr=rotator_pyr))

            # force PCG updates
            sp_pcg_subsystem.FlushCache(PCGSubsystem=pcg_subsystem)
            for bp_pcg_assembly_desc in bp_pcg_assembly_descs:
                for pcg_component in bp_pcg_assembly_desc["pcg_components"]:
                    pcg_component.Generate(bForce=True)

        with instance.end_frame():
            # read pixels from camera sensor
            if not args.skip_read_pixels:
                for component_desc in component_descs:
                    data_bundle = component_desc["component"].read_pixels()
                    component_desc["data"] = data_bundle["arrays"]["data"]

            gameplay_statics.SetGamePaused(bPaused=True)

        if not args.skip_read_pixels and not args.skip_save_images:
            save_images(images_dir=images_dir, frame_index=frame_index)
            frame_index = frame_index + 1

        # give PCG assemblies a chance to update
        for _ in range(num_frames_for_pcg_update):
            with instance.begin_frame():
                gameplay_statics.SetGamePaused(bPaused=False)
            with instance.end_frame():
                gameplay_statics.SetGamePaused(bPaused=True)

    #
    # unpause now that we're finished controlling the PCG assembly
    #

    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
    with instance.end_frame():
        pass

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    spear.log("Done.")
