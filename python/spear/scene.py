#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear
import sys


class Scene():
    def __init__(self, config, rpc_client):

        self.__config = config
        self.__rpc_client = rpc_client

        self._byte_order = self._get_byte_order()

    def set_object_locations(self, object_locations):

        self._begin_tick()
        self._tick()
        self._set_object_locations(object_locations)
        self._end_tick()

    def set_object_rotations(self, object_rotations):

        self._begin_tick()
        self._tick()
        self._set_object_rotations(object_rotations)
        self._end_tick()

    def get_all_object_names(self):

        self._begin_tick()
        self._tick()
        object_names = self._get_all_object_names()
        self._end_tick()

        return object_names

    def get_object_locations(self, object_names):

        self._begin_tick()
        self._tick()
        object_locations = self._get_object_locations(object_names)
        self._end_tick()

        return object_locations

    def get_object_rotations(self, object_names):

        self._begin_tick()
        self._tick()
        object_rotations = self._get_object_rotations(object_names)
        self._end_tick()

        return object_rotations
        
    def get_all_object_locations(self):

        self._begin_tick()
        self._tick()
        all_object_locations = self._get_all_object_locations()
        self._end_tick()

        return all_object_locations

    def get_all_object_rotations(self):

        self._begin_tick()
        self._tick()
        all_object_rotations = self._get_all_object_rotations()
        self._end_tick()

        return all_object_rotations

    def _set_object_locations(self, object_locations):
        object_locations_ = {}
        for name, object_location in object_locations.items():
            assert isinstance(object_location, np.ndarray)
            assert object_location.dtype == np.dtype("f8")
            assert object_location.shape[0] == 3
            object_locations_[name] = object_location.data if self._byte_order is None else object_location.newbyteorder(self._byte_order).data
        self.__rpc_client.call("scene.set_object_locations", object_locations_)

    def _set_object_rotations(self, object_rotations):
        object_rotations_ = {}
        for name, object_location in object_rotations.items():
            assert isinstance(object_location, np.ndarray)
            assert object_location.dtype == np.dtype("f8")
            assert object_location.shape[0] == 3
            object_rotations_[name] = object_location.data if self._byte_order is None else object_location.newbyteorder(self._byte_order).data
        self.__rpc_client.call("scene.set_object_rotations", object_rotations_)

    def _get_all_object_names(self):
        return self.__rpc_client.call("scene.get_all_object_names")

    def _get_object_locations(self, object_names):
        object_locations_ = self.__rpc_client.call("scene.get_object_locations", object_names)
        dtype = np.dtype("f8") if self._byte_order is None else np.dtype("f8").newbyteorder(self._byte_order)
        return np.frombuffer(object_locations_, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_object_rotations(self, object_names):
        object_rotations_ = self.__rpc_client.call("scene.get_object_rotations", object_names)
        dtype = np.dtype("f8") if self._byte_order is None else np.dtype("f8").newbyteorder(self._byte_order)
        return np.frombuffer(object_rotations_, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_all_object_locations(self):
        all_object_locations_ = self.__rpc_client.call("scene.get_all_object_locations")
        dtype = np.dtype("f8") if self._byte_order is None else np.dtype("f8").newbyteorder(self._byte_order)
        return {name: np.frombuffer(object_location, dtype=dtype, count=-1) for name, object_location in all_object_locations_.items()}

    def _get_all_object_rotations(self):
        all_object_rotations_ = self.__rpc_client.call("scene.get_all_object_rotations")
        dtype = np.dtype("f8") if self._byte_order is None else np.dtype("f8").newbyteorder(self._byte_order)
        return {name: np.frombuffer(object_rotation, dtype=dtype, count=-1) for name, object_rotation in all_object_rotations_.items()}

    def _begin_tick(self):
        self.__rpc_client.call("begin_tick")

    def _tick(self):
        self.__rpc_client.call("tick")

    def _end_tick(self):
        self.__rpc_client.call("end_tick")

    def _get_byte_order(self):
        unreal_instance_byte_order = self.__rpc_client.call("get_byte_order")
        rpc_client_byte_order = sys.byteorder
        if unreal_instance_byte_order == rpc_client_byte_order:
            return None
        elif unreal_instance_byte_order == "little":
            return "<"
        elif unreal_instance_byte_order == "big":
            return ">"
        else:
            assert False
