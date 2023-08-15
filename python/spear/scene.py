#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear


class Scene():
    def __init__(self, config, simulation_controller):

        self.__config = config
        self.__simulation_controller = simulation_controller

        self.__byte_order = self.__simulation_controller.get_byte_order()

    def set_object_locations(self, object_locations):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        self._set_object_locations(object_locations)
        self.__simulation_controller.end_tick()

    def set_object_rotations(self, object_rotations):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        self._set_object_rotations(object_rotations)
        self.__simulation_controller.end_tick()

    def get_all_object_names(self):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        object_names = self._get_all_object_names()
        self.__simulation_controller.end_tick()

        return object_names

    def get_object_locations(self, object_names):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        object_locations = self._get_object_locations(object_names)
        self.__simulation_controller.end_tick()

        return object_locations

    def get_object_rotations(self, object_names):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        object_rotations = self._get_object_rotations(object_names)
        self.__simulation_controller.end_tick()

        return object_rotations
        
    def get_all_object_locations(self):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        all_object_locations = self._get_all_object_locations()
        self.__simulation_controller.end_tick()

        return all_object_locations

    def get_all_object_rotations(self):

        self.__simulation_controller.begin_tick()
        self.__simulation_controller.tick()
        all_object_rotations = self._get_all_object_rotations()
        self.__simulation_controller.end_tick()

        return all_object_rotations

    def _set_object_locations(self, object_locations):
        object_locations_ = {}
        for name, object_location in object_locations.items():
            assert isinstance(object_location, np.ndarray)
            assert object_location.dtype == np.dtype("f8")
            assert object_location.shape[0] == 3
            object_locations_[name] = object_location.data if self.__byte_order is None else object_location.newbyteorder(self.__byte_order).data
        self.__simulation_controller.rpc_client.call("scene.set_object_locations", object_locations_)

    def _set_object_rotations(self, object_rotations):
        object_rotations_ = {}
        for name, object_location in object_rotations.items():
            assert isinstance(object_location, np.ndarray)
            assert object_location.dtype == np.dtype("f8")
            assert object_location.shape[0] == 3
            object_rotations_[name] = object_location.data if self.__byte_order is None else object_location.newbyteorder(self.__byte_order).data
        self.__simulation_controller.rpc_client.call("scene.set_object_rotations", object_rotations_)

    def _get_all_object_names(self):
        return self.__simulation_controller.rpc_client.call("scene.get_all_object_names")

    def _get_object_locations(self, object_names):
        object_locations_ = self.__simulation_controller.rpc_client.call("scene.get_object_locations", object_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(object_locations_, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_object_rotations(self, object_names):
        object_rotations_ = self.__simulation_controller.rpc_client.call("scene.get_object_rotations", object_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(object_rotations_, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_all_object_locations(self):
        all_object_locations_ = self.__simulation_controller.rpc_client.call("scene.get_all_object_locations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(object_location, dtype=dtype, count=-1) for name, object_location in all_object_locations_.items()}

    def _get_all_object_rotations(self):
        all_object_rotations_ = self.__simulation_controller.rpc_client.call("scene.get_all_object_rotations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(object_rotation, dtype=dtype, count=-1) for name, object_rotation in all_object_rotations_.items()}
