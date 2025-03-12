#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear

class NavigationService():
    def __init__(self, entry_point_caller, shared_memory_service):
        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service

    def get_nav_data_for_agent_name(self, navigation_system, agent_name):
        return self._entry_point_caller.call("navigation_service.get_nav_data_for_agent_name", navigation_system, agent_name)

    def get_random_points(self, navigation_data, num_points, out_array=np.array([])):

        # call function
        return_value_packed_array = self._entry_point_caller.call(
            "navigation_service.get_random_points",
            navigation_data,
            num_points,
            spear.to_packed_array(array=out_array, byte_order=self._shared_memory_service.unreal_instance_byte_order, usage_flags=["Arg", "ReturnValue"]))

        # get the shared memory handle for out_array
        return_value_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(arrays=[out_array], usage_flags=["Arg", "ReturnValue"])

        # convert return value to a NumPy array and return
        return spear.to_array(
            packed_array=return_value_packed_array,
            byte_order=self._shared_memory_service.unreal_instance_byte_order,
            usage_flags=["Arg", "ReturnValue"],
            shared_memory_handles=return_value_shared_memory_handles)
