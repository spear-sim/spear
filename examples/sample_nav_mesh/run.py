#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import numpy as np
import os
import pprint
import spear


num_points = 100


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    with instance.begin_frame():

        # find GetActorScale3D and SetActorScale3D functions
        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        set_actor_scale_3d_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="SetActorScale3D")

        # get axes uclass so we can spawn it later
        bp_axes_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Axes.BP_Axes_C")

        # since we're not using bp_axes_uclass in this frame, we need to explicitly prevent garbage collection
        instance.unreal_service.add_object_to_root(uobject=bp_axes_uclass)

        # get navigation system
        navigation_system_v1_static_class = instance.unreal_service.get_static_class(class_name="UNavigationSystemV1")
        get_navigation_system_func = instance.unreal_service.find_function_by_name(uclass=navigation_system_v1_static_class, function_name="GetNavigationSystem")
        navigation_system_v1_default_object = instance.unreal_service.get_default_object(uclass=navigation_system_v1_static_class, create_if_needed=False)
        return_values = instance.unreal_service.call_function(uobject=navigation_system_v1_default_object, ufunction=get_navigation_system_func)
        navigation_system = spear.to_handle(string=return_values["ReturnValue"])
        spear.log("navigation_system_v1_static_class: ", navigation_system_v1_static_class)
        spear.log("navigation_system_v1_default_object: ", navigation_system_v1_default_object)
        spear.log("navigation_system: ", navigation_system)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=navigation_system))

        # get navigation data
        navigation_data = instance.navigation_service.get_nav_data_for_agent_name(navigation_system=navigation_system, agent_name="Default")
        spear.log("navigation_data: ", navigation_data)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=navigation_data))

        # sample random points
        points = instance.navigation_service.get_random_points(
            navigation_data=navigation_data, num_points=num_points)
        spear.log("points: ")
        spear.log_no_prefix(points)

        # create shared memory for sampling random points, and allocate twice as much space as necessary for
        # demonstration purposes
        shared_memory_handle = instance.shared_memory_service.create_shared_memory_region(
            shared_memory_name="smem:random_points", num_bytes=2*num_points*3*8, usage_flags=["Arg", "ReturnValue"])

        # create NumPy array backed by the shared memory region we created
        shared = np.ndarray(shape=(2*num_points, 3), dtype=np.float64, buffer=shared_memory_handle["buffer"])
        shared[:] = np.zeros_like(shared)
        spear.log("shared: ")
        spear.log_no_prefix(shared)

        # if a shared NumPy array is passed as input, it will be used to store the output data; the function
        # will still return a NumPy array, but the returned array will be backed by the same memory as the
        # input array
        points = instance.navigation_service.get_random_points(
            navigation_data=navigation_data, num_points=num_points, out_array=spear.to_shared(array=shared, shared_memory_handle=shared_memory_handle))
        spear.log("points: ")
        spear.log_no_prefix(points)
        spear.log("shared: ")
        spear.log_no_prefix(shared)

    with instance.end_frame():
        pass

    # spawn axes
    for i in range(num_points):
        with instance.begin_frame():
            bp_axes = instance.unreal_service.spawn_actor_from_class(uclass=bp_axes_uclass, location={"X": points[i,0], "Y": points[i,1], "Z": points[i,2]})
            instance.unreal_service.call_function(uobject=bp_axes, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 0.75, "Y": 0.75, "Z": 0.75}})
        with instance.end_frame():
            pass

    # get random reachable points
    with instance.begin_frame():
        points = instance.navigation_service.get_random_points(
            navigation_data=navigation_data, num_points=1)
        spear.log("points: ")
        spear.log_no_prefix(points)

        reachable_points = instance.navigation_service.get_random_reachable_points_in_radius(
            navigation_data=navigation_data, num_points=num_points, origin_points=points, radius=100.0, out_array=spear.to_shared(array=shared, shared_memory_handle=shared_memory_handle))
        spear.log("reachable_points: ")
        spear.log_no_prefix(reachable_points)

    with instance.end_frame():
        pass

    # spawn axes
    for i in range(num_points):
        with instance.begin_frame():
            bp_axes = instance.unreal_service.spawn_actor_from_class(uclass=bp_axes_uclass, location={"X": reachable_points[i,0], "Y": reachable_points[i,1], "Z": reachable_points[i,2]})
            instance.unreal_service.call_function(uobject=bp_axes, ufunction=set_actor_scale_3d_func, args={"NewScale3D": {"X": 0.75, "Y": 0.75, "Z": 0.75}})
        with instance.end_frame():
            pass

    # find paths
    with instance.begin_frame():
        start_points = instance.navigation_service.get_random_points(
            navigation_data=navigation_data, num_points=1)
        spear.log("start_points: ")
        spear.log_no_prefix(start_points)

        end_points = instance.navigation_service.get_random_reachable_points_in_radius(
            navigation_data=navigation_data, num_points=1, origin_points=start_points, radius=2000.0)
        spear.log("end_points: ")
        spear.log_no_prefix(end_points)

        paths = instance.navigation_service.find_paths(
            navigation_system=navigation_system, navigation_data=navigation_data, num_paths=1, start_points=start_points, end_points=end_points)
        spear.log("paths: ")
        for p in paths:
            spear.log_no_prefix(p)

    with instance.end_frame():
        pass

    # destroy shared memory region and re-enable garbage collection for bp_axes_uclass
    with instance.begin_frame():
        instance.shared_memory_service.destroy_shared_memory_region(shared_memory_handle=shared_memory_handle)
        instance.unreal_service.remove_object_from_root(bp_axes_uclass)
    with instance.end_frame():
        pass

    spear.log("Done.")
