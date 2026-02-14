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

num_villagers_to_spawn = 1
num_build_target_candidate_locations = 100
num_frames = 100

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
        images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
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

    #
    # set up camera
    #

    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # get player controller
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)

        # get character
        player = player_controller.K2_GetPawn()

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

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    #
    # execute warm-up frames to give Unreal's default auto-exposure settings a chance to settle down
    #

    for _ in range(30):
        instance.flush()

    #
    # execute game logic
    #

    with instance.begin_frame():

        # get classes
        bpf_shared_uclass = game.unreal_service.load_class(uclass="UBlueprintFunctionLibrary", name="/Game/Blueprint/Core/Player/BPF_Shared.BPF_Shared_C")
        build_target_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Blueprint/Interactable/Building/BPC_EndGame.BPC_EndGame_C")
        interactable_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Blueprint/Interactable/BP_Interactable.BP_Interactable_C")
        villager_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Blueprint/Villagers/BP_Villager.BP_Villager_C")

        # get UnrealObject instances
        kismet_material_library = game.get_unreal_object(uclass="UKismetMaterialLibrary")
        navigation_system_v1 = game.get_unreal_object(uclass="UNavigationSystemV1")
        sp_navigation_system_v1 = game.get_unreal_object(uclass="USpNavigationSystemV1")
        bpf_shared = game.get_unreal_object(uclass=bpf_shared_uclass)

        # get game mode
        game_mode = gameplay_statics.GetGameMode()
        # game_mode.print_debug_info()

        # get navigation data
        navigation_system = navigation_system_v1.GetNavigationSystem()
        navigation_data = sp_navigation_system_v1.GetNavDataForAgentName(NavigationSystem=navigation_system, AgentName="Main")

        #
        # use call(...) when a function or argument name has spaces, and use get_property_value(...) or set_property_value(...)
        # when a property name has spaces; spaces with names in them can occur when functions and properties
        # are defined in Blueprints
        #

        # get function and property names
        # bpf_shared.print_debug_info()
        # game_mode.print_debug_info()
        # player.print_debug_info()

        # update cursor to be invisible to make our screenshots better
        cursor = game.unreal_service.get_component_by_name(actor=player, component_name="DefaultSceneRoot.Cursor", uclass="UStaticMeshComponent")
        cursor_visible = cursor.bVisible.get()
        cursor.bVisible = False

        # add resources so we have enough resources to build

        resource = {"Resource": "Food",  "Value": 1000}
        spear.log("Add resource: ", resource)
        game_mode.call(function_name="Add Resource", args=resource)

        resource = {"Resource": "Wood",  "Value": 1000}
        spear.log("Add resource: ", resource)
        game_mode.call(function_name="Add Resource", args=resource)

        resource = {"Resource": "Stone",  "Value": 1000}
        spear.log("Add resource: ", resource)
        game_mode.call(function_name="Add Resource", args=resource)

        # spawn villagers
        for _ in range(num_villagers_to_spawn):
            spear.log("Spawn villager.")
            game_mode.call(function_name="Spawn Villager")

        spear.log("Begin build.")

        # begin build
        player.call(function_name="Switch Build Mode", args={"Switch To Build Mode?": True})
        player.call(function_name="BeginBuild", args={"Target Class": build_target_uclass, "Resource Cost": {"Food": 0, "Wood": 500, "Stone": 500}})

        # get the temp build target object (i.e., the object that is visible in build mode but hasn't been permanently placed yet)
        temp_build_target = player.Spawn.get()

        # get candidate spawn locations and sort by distance to player
        player_location = spear.to_numpy_array_from_vector(vector=player.K2_GetActorLocation())
        spawn_candidate_locations = game.navigation_service.get_random_points(navigation_data=navigation_data, num_points=num_build_target_candidate_locations)
        spawn_candidate_locations[:,2] = 0.0
        sorted_indices = np.argsort(np.linalg.norm(spawn_candidate_locations - player_location, axis=1))
        spawn_candidate_locations_sorted = spawn_candidate_locations[sorted_indices]

        # try to find an empty location for the build target object
        can_spawn = False
        for spawn_candidate_location in spawn_candidate_locations_sorted:

            # sanity check that we are proceeding in order of increasing distance to the player
            distance = np.linalg.norm(spawn_candidate_location - player_location)

            # set temp build object location to a quantized candidate location
            return_values = bpf_shared.call(function_name="Convert To Stepped Pos", args={"A": spear.to_vector_from_numpy_array(array=spawn_candidate_location)}, as_dict=True)
            spawn_candidate_location_stepped = return_values["NewParam"]
            temp_build_target.K2_SetActorLocation(NewLocation=spawn_candidate_location_stepped)

            # test if the build object is overlapping any interactable actors 
            return_values = temp_build_target.GetOverlappingActors(ClassFilter=interactable_uclass, as_dict=True)
            num_overlapping_actors = len(return_values["OverlappingActors"])

            # test if all corners of the build object are in the nav mesh
            return_values = player.call(function_name="Corners in Nav", as_dict=True)
            corners_in_nav = return_values["NewParam"]

            # decide whether or not we can place the build object
            can_spawn = num_overlapping_actors == 0 and corners_in_nav

            spear.log(f"Candidate location: {temp_build_target.K2_GetActorLocation()} (distance={distance:.1f}, num_overlapping_actors={num_overlapping_actors}, corners_in_nav={corners_in_nav}, can_spawn={can_spawn})")

            if can_spawn:
                spear.log(f"Found a location to spawn the build target object.")
                break

        if not can_spawn:
            spear.log("Couldn't find a location to spawn the build target object. Giving up...")
            assert False

        spear.log("End build.")

        # end build
        player.set_property_value(property_name="Can Drop", property_value=can_spawn)
        player.call(function_name="Spawn Build Target")
        player.call(function_name="Destroy Spawn")
        player.call(function_name="Switch Build Mode", args={"Switch To Build Mode?": False})

        # get villagers
        villagers = game.unreal_service.find_actors_by_class(uclass=villager_uclass)
        assert len(villagers) > 0

        # get function and property names
        # villager.print_debug_info()

        # get build target
        build_targets = game.unreal_service.find_actors_by_class(uclass=build_target_uclass)
        assert len(build_targets) > 0
        build_target = build_targets[0]

        # assign villager to build target
        for i, villager in enumerate(villagers):
            spear.log(f"Assigning villager {i} to build building.")
            villager.Action(NewParam=build_target)

    with instance.end_frame():
        pass

    spear.log(f"Rendering images...")

    #
    # initialize frame counter
    #

    frame_index = 0

    #
    # render images
    #

    for i in range(num_frames):
        with instance.begin_frame():
            view_target_pov = player_camera_manager.ViewTarget.POV.get()

            fov = view_target_pov["fOV"]*math.pi/180.0
            half_fov = fov/2.0
            half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"]) # this adjustment is necessary to compute an FOV value that matches the game viewport
            fov_adjusted = half_fov_adjusted*2.0
            fov_adjusted_degrees = fov_adjusted*180.0/math.pi

            bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
            bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

            for component_desc in component_descs:
                component_desc["component"].FOVAngle = fov_adjusted_degrees

        with instance.end_frame():

            # read pixels from camera sensor
            if not args.skip_read_pixels:
                for component_desc in component_descs:
                    data_bundle = component_desc["component"].read_pixels()
                    component_desc["data"] = data_bundle["arrays"]["data"]

        if not args.skip_read_pixels and not args.skip_save_images:
            save_images(images_dir=images_dir, frame_index=frame_index)
            frame_index = frame_index + 1

    #
    # clean up game logic
    #

    with instance.begin_frame():
        cursor.bVisible = cursor_visible
    with instance.end_frame():
        pass

    spear.log("Done.")
