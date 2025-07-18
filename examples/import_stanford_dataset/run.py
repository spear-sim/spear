#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import time


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        # find functions
        actor_static_class = game.unreal_service.get_static_class(class_name="AActor")
        set_actor_scale_3d_func = game.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="SetActorScale3D")

        # spawn objects
        bp_bunny_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Bunny.BP_Bunny_C")
        bp_happy_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Happy.BP_Happy_C")
        bp_dragon_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Dragon.BP_Dragon_C")

        bp_bunny_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_bunny_uclass, location={"X": 80.0, "Y": 210.0, "Z": 15.0})
        bp_happy_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_happy_uclass, location={"X": 80.0, "Y": 280.0, "Z": 5.0})
        bp_dragon_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_dragon_uclass, location={"X": 80.0, "Y": 350.0, "Z": 5.0})

        # set scale
        game.unreal_service.call_function(uobject=bp_bunny_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})
        game.unreal_service.call_function(uobject=bp_happy_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})
        game.unreal_service.call_function(uobject=bp_dragon_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})

    with instance.end_frame():
        pass

    spear.log("Done.")
