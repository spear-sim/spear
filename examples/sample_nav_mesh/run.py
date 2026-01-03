#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
    game = instance.get_game()

    with instance.begin_frame():

        # get axes uclass so we can spawn it later
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")

        # since we're not using bp_axes_uclass in this frame, we need to explicitly prevent garbage collection
        game.unreal_service.add_object_to_root(uobject=bp_axes_uclass)

        # get navigation system
        navigation_system_v1 = game.get_unreal_object(uclass="UNavigationSystemV1")
        sp_navigation_system_v1 = game.get_unreal_object(uclass="USpNavigationSystemV1")
        navigation_system = navigation_system_v1.GetNavigationSystem()
        supports_rebuilding = navigation_system.bSupportRebuilding.get()                                   # get bSupportRebuilding flag
        navigation_system.bSupportRebuilding = True                                                        # needs to be set at runtime to force a rebuild
        sp_navigation_system_v1.Build(NavigationSystem=navigation_system)                                  # force a rebuild
        sp_navigation_system_v1.AddNavigationBuildLock(NavigationSystem=navigation_system, Flags="Custom") # prevent nav mesh from updating from now on
        navigation_system.bSupportRebuilding = supports_rebuilding                                         # restore bSupportRebuilding flag

        # get navigation data
        navigation_data = sp_navigation_system_v1.GetNavDataForAgentName(NavigationSystem=navigation_system, AgentName="Default")
        spear.log("navigation_data: ", navigation_data)
        pprint.pprint(navigation_data.get_properties())

        # sample random points
        points = game.navigation_service.get_random_points(navigation_data=navigation_data, num_points=num_points)
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
        points = game.navigation_service.get_random_points(
            navigation_data=navigation_data,
            num_points=num_points,
            out_array=spear.to_shared(array=shared, shared_memory_handle=shared_memory_handle))
        spear.log("points: ")
        spear.log_no_prefix(points)
        spear.log("shared: ")
        spear.log_no_prefix(shared)

    with instance.end_frame():
        pass

    # spawn axes
    for i in range(num_points):
        with instance.begin_frame():
            bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": points[i,0], "Y": points[i,1], "Z": points[i,2]})
            bp_axes.SetActorScale3D(NewScale3D={"X": 0.75, "Y": 0.75, "Z": 0.75})
        with instance.end_frame():
            pass

    # get random reachable points
    with instance.begin_frame():
        points = game.navigation_service.get_random_points(
            navigation_data=navigation_data, num_points=1)
        spear.log("points: ")
        spear.log_no_prefix(points)

        reachable_points = game.navigation_service.get_random_reachable_points_in_radius(
            navigation_data=navigation_data,
            num_points=num_points,
            origin_points=points,
            radius=100.0,
            out_array=spear.to_shared(array=shared, shared_memory_handle=shared_memory_handle))
        spear.log("reachable_points: ")
        spear.log_no_prefix(reachable_points)

    with instance.end_frame():
        pass

    # spawn axes
    for i in range(num_points):
        with instance.begin_frame():
            bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": reachable_points[i,0], "Y": reachable_points[i,1], "Z": reachable_points[i,2]})
            bp_axes.SetActorScale3D(NewScale3D={"X": 0.75, "Y": 0.75, "Z": 0.75})
        with instance.end_frame():
            pass

    # find paths
    with instance.begin_frame():
        start_points = game.navigation_service.get_random_points(
            navigation_data=navigation_data,
            num_points=1)
        spear.log("start_points: ")
        spear.log_no_prefix(start_points)

        end_points = game.navigation_service.get_random_reachable_points_in_radius(
            navigation_data=navigation_data,
            num_points=1,
            origin_points=start_points,
            radius=2000.0)
        spear.log("end_points: ")
        spear.log_no_prefix(end_points)

        paths = game.navigation_service.find_paths(
            navigation_system=navigation_system,
            navigation_data=navigation_data,
            num_paths=1,
            start_points=start_points,
            end_points=end_points)
        spear.log("paths: ")
        for p in paths:
            spear.log_no_prefix(p)

    with instance.end_frame():
        pass

    # destroy shared memory region and re-enable garbage collection for bp_axes_uclass
    with instance.begin_frame():
        instance.shared_memory_service.destroy_shared_memory_region(shared_memory_handle=shared_memory_handle)
        game.unreal_service.remove_object_from_root(bp_axes_uclass)
    with instance.end_frame():
        pass

    spear.log("Done.")
