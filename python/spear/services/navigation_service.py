#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear

class NavigationService(spear.utils.func_utils.Service):
    def __init__(self, entry_point_caller, shared_memory_service, create_children=True):

        self._entry_point_caller = entry_point_caller
        self._shared_memory_service = shared_memory_service
        super().__init__(entry_point_caller, create_children) # do this after initializing local state

    def create_child(self, entry_point_caller):
        return NavigationService(entry_point_caller=entry_point_caller, shared_memory_service=self._shared_memory_service, create_children=False)

    def get_nav_data_for_agent_name(self, navigation_system, agent_name):
        return self._entry_point_caller.call_on_game_thread("uint64_t", "get_nav_data_for_agent_name", None, navigation_system, agent_name)

    # The caller must ensure that out_array remains valid until future.get() has been called for the future
    # that might be returned by this function.

    def get_random_points(
        self,
        navigation_data,
        num_points,
        query_owner=0,
        filter_classes=np.array([], dtype=np.uint64),
        queriers=np.array([], dtype=np.uint64),
        out_array=np.array([], dtype=np.float64)):

        packed_arrays = spear.utils.func_utils.to_packed_arrays(
            arrays={"filter_classes": filter_classes, "queriers": queriers},
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg"])
        out_packed_array = spear.utils.func_utils.to_packed_array(
            array=out_array,
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg", "ReturnValue"])

        # define convert func
        def convert_func(result_packed_array, out_array=out_array):
            result_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(
                arrays=[out_array],
                usage_flags=["Arg", "ReturnValue"])
            result = spear.utils.func_utils.to_array(
                packed_array=result_packed_array,
                src_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
                usage_flags=["Arg", "ReturnValue"],
                shared_memory_handles=result_shared_memory_handles)
            return result

        return self._entry_point_caller.call_on_game_thread(
            "PackedArray",
            "get_random_points",
            convert_func,
            navigation_data,
            num_points,
            query_owner,
            packed_arrays,
            out_packed_array)

    # The caller must ensure that out_array remains valid until future.get() has been called for the future
    # that might be returned by this function.

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

        packed_arrays = spear.utils.func_utils.to_packed_arrays(
            arrays={"origin_points": origin_points, "radii": radii, "filter_classes": filter_classes, "queriers": queriers},
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg"])
        out_packed_array = spear.utils.func_utils.to_packed_array(
            array=out_array,
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg", "ReturnValue"])

        # define convert func
        def convert_func(result_packed_array, out_array=out_array):
            result_shared_memory_handles = self._shared_memory_service.get_shared_memory_handles_from_arrays(
                arrays=[out_array],
                usage_flags=["Arg", "ReturnValue"])
            result = spear.utils.func_utils.to_array(
                packed_array=result_packed_array,
                src_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
                usage_flags=["Arg", "ReturnValue"],
                shared_memory_handles=result_shared_memory_handles)
            return result

        return self._entry_point_caller.call_on_game_thread(
            "PackedArray",
            "get_random_reachable_points_in_radius",
            convert_func,
            navigation_data,
            num_points,
            query_owner,
            packed_arrays,
            out_packed_array)

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

        packed_arrays = spear.utils.func_utils.to_packed_arrays(
            arrays={
                "start_points": start_points,
                "end_points": end_points,
                "filter_classes": filter_classes,
                "queriers": queriers,
                "cost_limits": cost_limits,
                "require_navigable_end_locations": require_navigable_end_locations},
            dest_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
            usage_flags=["Arg"])

        nav_agent_property_strings = spear.utils.func_utils.to_json_strings(nav_agent_properties)

        # define convert func
        def convert_func(result_packed_arrays):
            return_values = spear.utils.func_utils.to_arrays(
                packed_arrays=result_packed_arrays,
                src_byte_order=self._entry_point_caller.engine_service.get_byte_order(),
                usage_flags=None,
                shared_memory_handles=None)

            # return paths as a list of NumPy arrays rather than as a points array and an indices array
            points = return_values["points"]
            indices = return_values["indices"]
            paths = []
            for i in range(indices.shape[0] - 1):
                paths.append(points[indices[i]:indices[i + 1]])
            paths.append(points[indices[-1]:])

            return paths

        return self._entry_point_caller.call_on_game_thread(
            "std::map<std::string, PackedArray>",
            "find_paths",
            convert_func,
            navigation_system,
            navigation_data,
            num_paths,
            nav_agent_interface,
            packed_arrays,
            nav_agent_property_strings,
            path_finding_mode_strings)
