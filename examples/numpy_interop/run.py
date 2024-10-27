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

        # Create a shared memory region for passing array data to an Unreal object as efficiently as possible.
        # In typical use cases, Python code is responsible for defining shared memory regions for passing
        # arguments to Unreal objects, and Unreal objects are responsible for defining shared memory regions
        # for passing return values to Python. The name "smem_action" needs to be unique across all Python
        # code interacting with SpFuncService.
        action_shared_memory_handle = instance.sp_func_service.create_shared_memory_region(num_bytes=1024, shared_memory_name="smem_action")

        # Get the default ASpDebugWidget object. In this example, we're calling a custom function on an
        # Unreal actor, but we can use the same interface to call custom functions on Unreal components.
        sp_debug_widget_static_class = instance.unreal_service.get_static_class(class_name="ASpDebugWidget")
        sp_debug_widget_default_object = instance.unreal_service.get_default_object(uclass=sp_debug_widget_static_class, create_if_needed=False)

        # Create handles to any shared memory regions created by the Unreal object. SpFuncService.call_function(...)
        # will use these handles internally to access data returned via shared memory.
        sp_debug_widget_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_uobject(uobject=sp_debug_widget_default_object)

        # Create a numpy array.
        action = np.array([0.0, 1.0, 2.0])

        # Create numpy array backed by the shared memory region we created.
        action_shared = np.ndarray(shape=(3,), dtype=np.float64, buffer=action_shared_memory_handle["buffer"])
        action_shared[:] = [3.0, 4.0, 5.0]

        # Prepare args for calling a custom function on our object. Note that any array backed by shared
        # memory needs to be wrapped with spear.to_shared(...) when passing it to SpFuncService.call_function(...).
        # Otherwise it will be treated as a regular array, and will be sent to the Unreal object via a slower
        # code path.
        arrays = {
            "action": action,
            "action_shared": spear.to_shared(array=action_shared, shared_memory_name="smem_action")}
        unreal_objs = {
            "in_location": {"X": 6.0, "Y": 7.0, "Z": 8.0},
            "in_rotation": {"Pitch": 9.0, "Yaw": 10.0, "Roll": 11.0}}
        info = "Hello world"

        spear.log("arrays:      ", arrays)
        spear.log("unreal_objs: ", unreal_objs)
        spear.log("info:        ", info)

        # Call the custom function named "hello_world" on our Unreal object.
        return_values = instance.sp_func_service.call_function(
            sp_debug_widget_default_object,
            "hello_world",
            arrays=arrays,
            unreal_objs=unreal_objs,
            info=info,
            uobject_shared_memory_handles=sp_debug_widget_shared_memory_handles)

        spear.log("return_values: ", return_values)

        # Destroy handles to the shared memory regions created by the Unreal object.
        instance.sp_func_service.destroy_shared_memory_handles_for_uobject(shared_memory_handles=sp_debug_widget_shared_memory_handles)

        # Destroy the shared memory region we created ourselves.
        instance.sp_func_service.destroy_shared_memory_region(shared_memory_name="smem_action")

    with instance.end_frame():
        pass

    spear.log("Done.")
