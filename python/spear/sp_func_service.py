#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import spear

class SpFuncService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client
        self._unreal_instance_byte_order = self._rpc_client.call("engine_service.get_byte_order")

    def call_function(self, uobject, function_name, packed_arrays={}, unreal_objs={}, info=""):

        # If an arg is a string, then don't convert. If an arg is a spear.Ptr, then use spear.Ptr.to_string()
        # to convert. If an arg is any other type, then assume it is valid JSON and use json.dumps(...) to
        # convert.
        unreal_obj_strings = {}
        for unreal_obj_name, unreal_obj in unreal_objs.items():
            if isinstance(unreal_obj, str):
                unreal_obj_string = unreal_obj
            elif isinstance(unreal_obj, spear.Ptr):
                unreal_obj_string = unreal_obj.to_string()
            else:
                unreal_obj_string = json.dumps(unreal_obj)
            unreal_obj_strings[unreal_obj_name] = unreal_obj_string

        args = {"packed_arrays": packed_arrays, "unreal_obj_strings": unreal_obj_strings, "info": info}
        sp_func_return_values = self._rpc_client.call("sp_func_service.call_function", uobject, function_name, args)

        # Try to parse each return value string as JSON, and if that doesn't work, then return the string
        # directly. If the returned string is intended to be a handle, then the user can get it as a handle
        # by calling spear.to_handle(...).
        unreal_objs = {}
        for unreal_obj_name, unreal_obj_string in sp_func_return_values["unreal_obj_strings"].items():
            try:
                unreal_obj = json.loads(unreal_obj_string)
            except:
                unreal_obj = unreal_obj_string
            unreal_objs[unreal_obj_name] = unreal_obj

        return_values = {"packed_arrays": sp_func_return_values["packed_arrays"], "unreal_objs": unreal_objs, "info": sp_func_return_values["info"]}

        return return_values

    def get_shared_memory_views(self, uobject):
        return self._rpc_client.call("sp_func_service.get_shared_memory_views", uobject)
