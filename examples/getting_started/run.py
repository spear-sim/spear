#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear


if __name__ == "__main__":

    # create an instance representing a UE application; this will either launch a new UE application or
    # connect to an existing one depending on how user_config.yaml is defined
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # the code in each instance.begin_frame() block executes sequentially at the beginning of a single UE
    # frame; as soon as each Python function returns, its side effects on the UE application are complete
    # and immediately observable; this programming model enables UE work to be executed deterministically
    # in a single frame, even when there are complex data dependencies among work items
    with instance.begin_frame():

        # spawn object
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

        # print all available functions and properties and other debug info for bp_axes
        bp_axes.print_debug_info()

        # get all object properties as nested Python dictionaries
        spear.log("bp_axes: ", bp_axes)
        spear.log("bp_axes.get_properties():")
        pprint.pprint(bp_axes.get_properties())

        # get scale by calling the AActor::GetActorScale3D C++ function
        scale = bp_axes.GetActorScale3D()
        spear.log("scale: ", scale)

        # set scale by calling the AActor::SetActorScale3D C++ function
        bp_axes.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})

        # get scale again to verify it has been updated
        scale = bp_axes.GetActorScale3D()
        spear.log("scale: ", scale)

        # get the RootComponent property on bp_axes as a Python object with its own functions
        root_component = bp_axes.RootComponent.get()

        # print all available functions and properties and other debug info for root_component
        root_component.print_debug_info()

        # get all object properties for root_component as nested Python dictionaries
        spear.log("root_component: ", root_component)
        spear.log("root_component.get_properties():")
        pprint.pprint(root_component.get_properties())

    # each instance.begin_frame() block must be paired with a corresponding instance.end_frame() block; the
    # code in the end_frame() block executes in the same UE frame as the begin_frame() block, except it
    # executes at the end of the frame instead of the beginning; this is useful, e.g., in embodied AI
    # applications, where it is desirable to input a control action at the beginning of a simulation frame
    # and collect an observation at the end of the same frame
    with instance.end_frame():
        pass

    spear.log("Done.")
