#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import numpy as np
import os
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        # Create a shared memory region for passing array data to an Unreal object as efficiently as possible.
        # In typical use cases, Python code is responsible for defining shared memory regions for passing
        # arguments to Unreal objects, and Unreal objects are responsible for defining shared memory regions
        # for passing return values to Python. The name "smem:action" needs to be unique across all Python
        # code interacting with instance.shared_memory_service.
        shared_memory_handle = instance.shared_memory_service.create_shared_memory_region(shared_memory_name="smem:action", num_bytes=1024, usage_flags=["Arg"])

        # Spawn an ASpDebugManager object. In this example, we're calling a custom function on an actor
        # type, but we can use the same interface to call custom functions on Unreal components. See
        # examples/render_image/run.py for more details.
        sp_debug_manager = game.unreal_service.spawn_actor(uclass="ASpDebugManager")
        sp_debug_manager.Initialize()
        sp_debug_manager.initialize_sp_funcs() # need to call initialize_sp_funcs() after calling Initialize() because hello_world() is registered during Initialize()

        # Create a NumPy array.
        action = np.array([0.0, 1.0, 2.0])

        # Create NumPy array backed by the shared memory region we created.
        action_shared = np.ndarray(shape=(3,), dtype=np.float64, buffer=shared_memory_handle["buffer"])
        action_shared[:] = [3.0, 4.0, 5.0]

        # Prepare args for calling a custom function on our object. Note that any array backed by shared
        # memory needs to be wrapped with spear.to_shared(...) when passing it to a custom function.
        # Otherwise it will be treated as a regular NumPy array, and will be sent to the Unreal object via a
        # slower code path.
        arrays = {
            "action": action,
            "action_shared": spear.to_shared(array=action_shared, shared_memory_handle=shared_memory_handle)}
        unreal_objs = {
            "in_location": {"X": 6.0, "Y": 7.0, "Z": 8.0},
            "in_rotation": {"Pitch": 9.0, "Yaw": 10.0, "Roll": 11.0}}
        info = "Hello world"

        spear.log("arrays:      ", arrays)
        spear.log("unreal_objs: ", unreal_objs)
        spear.log("info:        ", info)

        # Call the SpFunc named "hello_world" on our Unreal object.
        return_values = sp_debug_manager.hello_world(arrays=arrays, unreal_objs=unreal_objs, info=info)

        spear.log("return_values: ", return_values)
        spear.log('return_values["arrays"]["observation"].flags["ALIGNED"]:        ', return_values["arrays"]["observation"].flags["ALIGNED"])
        spear.log('return_values["arrays"]["observation_shared"].flags["ALIGNED"]: ', return_values["arrays"]["observation_shared"].flags["ALIGNED"])

        sp_debug_manager.terminate_sp_funcs()
        sp_debug_manager.Terminate()

        # Destroy our shared memory region.
        instance.shared_memory_service.destroy_shared_memory_region(shared_memory_handle=shared_memory_handle)

    with instance.end_frame():
        pass

    spear.log("Done.")
