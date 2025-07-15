#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear.func_utils


class NavigationService():
    def __init__(self, entry_point_caller, shared_memory_service, namespace):
        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service
        self._service_name = f"{namespace}.navigation_service"


    def get_nav_data_for_agent_name(self, navigation_system, agent_name):
        return self._entry_point_caller.call_on_game_thread_and_get_return_value(
            "uint64_t",
            f"{self._service_name}.get_nav_data_for_agent_name",
            navigation_system,
            agent_name)


    def get_random_points(
        self,
        navigation_data,
        num_points,
        query_owner=0,
        filter_classes=np.array([], dtype=np.uint64),
        queriers=np.array([], dtype=np.uint64),
        out_array=np.array([], dtype=np.float64)):

        packed_arrays = spear.func_utils.to_packed_arrays(arrays={"filter_classes": filter_classes, "queriers": queriers}, usage_flags=["Arg"])
        out_packed_array = spear.func_utils.to_packed_array(array=out_array, usage_flags=["Arg", "ReturnValue"])

        # call function
        return_value_packed_array = self._entry_point_caller.call_on_game_thread_and_get_converted_return_value(
            "PackedArray",
            f"{self._service_name}.get_random_points",
            navigation_data,
            num_points,
            query_owner,
            packed_arrays,
            out_packed_array)

        # convert return value to a NumPy array
        return_value_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(arrays=[out_array], usage_flags=["Arg", "ReturnValue"])
        return_value = spear.func_utils.to_array(packed_array=return_value_packed_array, usage_flags=["Arg", "ReturnValue"], shared_memory_handles=return_value_shared_memory_handles)

        return return_value


    def get_random_reachable_points_in_radius(
        self,
        navigation_data,
        num_points,
        origin_points,
        query_owner=0,
        radii=None,
        radius=None,
        filter_classes=np.array([], dtype=np.uint64),
        queriers=np.array([], dtype=np.uint64),
        out_array=np.array([], dtype=np.float64)):

        assert (radii is not None) + (radius is not None) == 1

        if radius is not None:
            radii = np.array([radius], dtype=np.float32)

        packed_arrays = spear.func_utils.to_packed_arrays(arrays={"origin_points": origin_points, "radii": radii, "filter_classes": filter_classes, "queriers": queriers}, usage_flags=["Arg"])
        out_packed_array = spear.func_utils.to_packed_array(array=out_array, usage_flags=["Arg", "ReturnValue"])

        # call function
        return_value_packed_array = self._entry_point_caller.call_on_game_thread_and_get_converted_return_value(
            "PackedArray",
            f"{self._service_name}.get_random_reachable_points_in_radius",
            navigation_data,
            num_points,
            query_owner,
            packed_arrays,
            out_packed_array)

        # convert return value to a NumPy array
        return_value_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(arrays=[out_array], usage_flags=["Arg", "ReturnValue"])
        return_value = spear.func_utils.to_array(packed_array=return_value_packed_array, usage_flags=["Arg", "ReturnValue"], shared_memory_handles=return_value_shared_memory_handles)

        return return_value


    def find_paths(
        self,
        navigation_system,
        navigation_data,
        num_paths,
        start_points,
        end_points,
        nav_agent_interface=0,
        filter_classes=np.array([], dtype=np.uint64),
        queriers=np.array([], dtype=np.uint64),
        cost_limits=np.array([], dtype=np.float64),
        require_navigable_end_locations=np.array([], dtype=np.uint8),
        nav_agent_properties=[],
        path_finding_mode_strings=[]):

        packed_arrays = spear.func_utils.to_packed_arrays(
            arrays={
                "start_points": start_points,
                "end_points": end_points,
                "filter_classes": filter_classes,
                "queriers": queriers,
                "cost_limits": cost_limits,
                "require_navigable_end_locations": require_navigable_end_locations},
            usage_flags=["Arg"])

        nav_agent_property_strings = spear.func_utils.to_json_strings(nav_agent_properties)

        # call function
        return_value_packed_arrays = self._entry_point_caller.call_on_game_thread_and_get_converted_return_value(
            "std::map<std::string, PackedArray>",
            f"{self._service_name}.find_paths",
            navigation_system,
            navigation_data,
            num_paths,
            nav_agent_interface,
            packed_arrays,
            nav_agent_property_strings,
            path_finding_mode_strings)

        # convert return values to NumPy arrays
        return_values = spear.func_utils.to_arrays(packed_arrays=return_value_packed_arrays)

        # return paths as a list of NumPy arrays rather than as a points array and an indices array
        points = return_values["points"]
        indices = return_values["indices"]
        paths = []
        for i in range(indices.shape[0] - 1):
            paths.append(points[indices[i]:indices[i + 1]])
        paths.append(points[indices[-1]:])

        return paths
