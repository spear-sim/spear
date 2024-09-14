#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear


if __name__ == "__main__":

    # create SPEAR instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    spear_instance = spear.Instance(config)

    spear_instance.engine_service.begin_tick()

    # find functions
    actor_static_class = spear_instance.unreal_service.get_static_class(class_name="AActor")
    set_actor_scale_3d_func = spear_instance.unreal_service.find_function_by_name(uclass=actor_static_class, name="SetActorScale3D")

    # spawn objects
    bp_bunny_uclass = spear_instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Bunny.BP_Bunny_C")
    bp_happy_uclass = spear_instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Happy.BP_Happy_C")
    bp_dragon_uclass = spear_instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/Stanford/Blueprints/BP_Dragon.BP_Dragon_C")

    bp_bunny_actor = spear_instance.unreal_service.spawn_actor_from_uclass(uclass=bp_bunny_uclass, location={"X": 80.0, "Y": 210.0, "Z": 15.0})
    bp_happy_actor = spear_instance.unreal_service.spawn_actor_from_uclass(uclass=bp_happy_uclass, location={"X": 80.0, "Y": 280.0, "Z": 5.0})
    bp_dragon_actor = spear_instance.unreal_service.spawn_actor_from_uclass(uclass=bp_dragon_uclass, location={"X": 80.0, "Y": 350.0, "Z": 5.0})

    # access object properties on the root component of bp_bunny_actor
    root_component_property_desc = spear_instance.unreal_service.find_property_by_name_on_uobject(uobject=bp_bunny_actor, name="RootComponent")
    root_component_string = spear_instance.unreal_service.get_property_value_as_string(property_desc=root_component_property_desc)
    root_component = spear_instance.unreal_service.to_handle(string=root_component_string)
    root_component_object_properties = spear_instance.unreal_service.get_object_properties_from_uobject(uobject=root_component)

    # print properties and helper objects
    spear.log(root_component_property_desc)
    spear.log(root_component_string)
    spear.log(root_component)
    spear.log(root_component_object_properties)

    # set scale
    spear_instance.unreal_service.call_function(uobject=bp_bunny_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})
    spear_instance.unreal_service.call_function(uobject=bp_happy_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})
    spear_instance.unreal_service.call_function(uobject=bp_dragon_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})

    spear_instance.engine_service.tick()
    spear_instance.engine_service.end_tick()

    # don't close the instance so we have a chance to navigate around the scene
    # spear_instance.close()

    spear.log("Done.")
