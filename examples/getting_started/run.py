#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        # spawn object
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

        # print functions and properties and other debug info
        bp_axes.print_debug_info()

        # get all object properties
        spear.log("bp_axes: ", bp_axes)
        spear.log("bp_axes.get_properties():")
        pprint.pprint(bp_axes.get_properties())

        # get scale
        scale = bp_axes.GetActorScale3D()
        spear.log("scale: ", scale)

        # set scale and get it again to verify that it has been updated
        bp_axes.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})
        scale = bp_axes.GetActorScale3D()
        spear.log("scale: ", scale)

        # access object properties on the root component of bp_axes
        root_component = bp_axes.RootComponent.get()

        # print functions and properties and other debug info
        root_component.print_debug_info()

        spear.log("root_component: ", root_component)
        spear.log("root_component.get_properties():")
        pprint.pprint(root_component.get_properties())

    with instance.end_frame():
        pass

    spear.log("Done.")
