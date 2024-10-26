#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear

if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():

        # find GetActorScale3D and SetActorScale3D functions
        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        get_actor_scale_3d_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="GetActorScale3D")
        set_actor_scale_3d_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="SetActorScale3D")

        # spawn object
        bp_axes_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes_actor = instance.unreal_service.spawn_actor_from_uclass(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

        # get scale
        return_values = instance.unreal_service.call_function(uobject=bp_axes_actor, ufunction=get_actor_scale_3d_func)
        spear.log("return_values: ", return_values)

        # set scale and get it again to verify that it updated
        instance.unreal_service.call_function(uobject=bp_axes_actor, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 4.0, "Y": 4.0, "Z": 4.0}})
        return_values = instance.unreal_service.call_function(uobject=bp_axes_actor, ufunction=get_actor_scale_3d_func)
        spear.log("return_values: ", return_values)

        # access object properties on the root component of bp_axes_actor
        root_component_property_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=bp_axes_actor, property_name="RootComponent")
        root_component_string = instance.unreal_service.get_property_value_as_string(property_desc=root_component_property_desc)
        root_component = spear.to_handle(string=root_component_string)
        root_component_object_properties = instance.unreal_service.get_object_properties_from_uobject(uobject=root_component)

        # print properties
        spear.log("RootComponent: ")
        pprint.pprint(root_component_object_properties)

    with instance.end_frame():
        pass

    spear.log("Done.")
