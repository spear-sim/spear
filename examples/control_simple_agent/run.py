#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os
import spear


num_steps = 150


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # spawn object and get components
        bp_sphere_agent_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_SphereAgent.BP_SphereAgent_C")
        bp_sphere_agent = game.unreal_service.spawn_actor(uclass=bp_sphere_agent_uclass, location={"X": -10.0, "Y": 280.0, "Z": 150.0})
        root_component = game.unreal_service.get_component_by_name(actor=bp_sphere_agent, component_name="DefaultSceneRoot", uclass="USceneComponent")
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_sphere_agent, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USceneComponent")

        #
        # We get the sphere component "by path" (instead of "by name") because this particular component gets
        # automatically relocated in the component hierarchy by Unreal at some point during BeginPlay().
        # Unreal performs this relocation operation because the component is physics-enabled. Searching by
        # path will look for the component at the path "DefaultSceneRoot.sphere_" (its original path before
        # getting relocated) and the path "sphere_" (its path after getting relocated).
        #
        # At this point in the code, we are certain that the relocation operation has already happened, so we
        # could just search by name using the name "sphere_". But this would be confusing. And we have a
        # dedicated C++ function for searching by path, which is helpful, e.g., inside an actor's BeginPlay()
        # method, since in BeginPlay() it isn't clear if a relocation operation has happened yet. As a matter
        # of readability, we choose to search by path here, so we can search for the component using it's
        # original path.
        #

        sphere_component = game.unreal_service.get_component_by_path(actor=bp_sphere_agent, component_path="DefaultSceneRoot.sphere_", uclass="USceneComponent")

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

        # show FPS
        game.unreal_service.execute_console_command("stat fps")

    with instance.end_frame():
        pass

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame can fix occasional render-to-texture initialization issues on macOS
    for i in range(1):
        instance.flush()

    # take a few steps
    for i in range(num_steps):

        # apply action
        with instance.begin_frame():

            # unpause
            gameplay_statics.SetGamePaused(bPaused=False)

            # add rotation
            root_component.K2_AddRelativeRotation(DeltaRotation={"Pitch": 0.0, "Yaw": 1.0, "Roll": 0.0})

            # add force
            rotator = root_component.K2_GetComponentRotation()
            R_world_from_component = spear.to_numpy_matrix_from_rotator(rotator=rotator, as_matrix=True)
            force_component = np.matrix([1000.0, 0.0, 0.0]).T
            force_world = R_world_from_component*force_component
            sphere_component.AddForce(Force=spear.to_vector_from_numpy_array(array=force_world))

        # get observation
        with instance.end_frame():

            # read pixels
            data_bundle = final_tone_curve_hdr_component.read_pixels()

            # pause
            gameplay_statics.SetGamePaused(bPaused=True)

        # show image in an OpenCV window
        cv2.imshow("final_tone_curve_hdr", data_bundle["arrays"]["data"])
        cv2.waitKey(0)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_sphere_agent)

    # close OpenCV window
    cv2.destroyAllWindows()

    spear.log("Done.")
