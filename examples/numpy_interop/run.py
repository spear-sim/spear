#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import numpy as np
import os
import spear

if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():

        # get the default ASpDebugWidget object
        sp_debug_widget_static_class = instance.unreal_service.get_static_class(class_name="ASpDebugWidget")
        sp_debug_widget_default_object = instance.unreal_service.get_default_object(uclass=sp_debug_widget_static_class, create_if_needed=False)

        # create handles to the object's shared memory regions
        shared_memory_handles = instance.sp_func_service.create_shared_memory_handles(sp_debug_widget_default_object)

        # create numpy array
        action = np.array([0.0, 1.0, 2.0])

        # create numpy array backed by shared memory
        action_shared = np.ndarray(shape=(3,), dtype=np.float64, buffer=shared_memory_handles["hello_shared_memory"]["buffer"])
        action_shared[:] = [3.0, 4.0, 5.0]

        # prepare args
        arrays = {"action": action, "action_shared": spear.to_shared(action_shared, "hello_shared_memory")}
        unreal_objs = {"in_location": {"X": 6.0, "Y": 7.0, "Z": 8.0}, "in_rotation": {"Pitch": 9.0, "Yaw": 10.0, "Roll": 11.0}}
        info = "Hello world"

        spear.log("arrays:      ", arrays)
        spear.log("unreal_objs: ", unreal_objs)
        spear.log("info:        ", info)

        # call "hello_world" function on debug widget
        return_values = instance.sp_func_service.call_function(
            sp_debug_widget_default_object, "hello_world", arrays=arrays, unreal_objs=unreal_objs, info=info, shared_memory_handles=shared_memory_handles)

        spear.log("return_values: ", return_values)

        # destroy shared memory handles
        instance.sp_func_service.destroy_shared_memory_handles(shared_memory_handles)

    with instance.end_frame():
        pass

    spear.log("Done.")
