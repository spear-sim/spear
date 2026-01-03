#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class EngineGlobalsServiceWrapper(spear.ServiceWrapper):
    def __init__(self, service, sp_func_service, unreal_service, config, parent_service_wrapper=None, create_children_service_wrappers=True):
        assert isinstance(service, EngineGlobalsService)

        # do this after initializing local state
        super().__init__(
            service=service,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config,
            parent_service_wrapper=parent_service_wrapper,
            create_children_service_wrappers=create_children_service_wrappers)

    def create_child_service_wrapper(self, service):
        assert self.is_top_level_service_wrapper() # this function should only be called from the top-level service wrapper
        return EngineGlobalsServiceWrapper(service=service, sp_func_service=self.sp_func_service, unreal_service=self.unreal_service, config=self.config, parent_service_wrapper=self, create_children_service_wrappers=False)

    def get_engine(self, as_handle=None, as_unreal_object=None, with_sp_funcs=None):
        result = self.service.get_engine()
        return self.to_handle_or_unreal_object(obj=result, as_handle=as_handle, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)


class EngineGlobalsService(spear.Service):
    def __init__(self, entry_point_caller, parent_service=None, create_children_services=True):

        # do this after initializing local state
        super().__init__(entry_point_caller=entry_point_caller, parent_service=parent_service, create_children_services=create_children_services)

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert self.is_top_level_service() # this function should only be called from the top-level service
        return EngineGlobalsService(entry_point_caller=entry_point_caller, parent_service=self, create_children_services=False)

    #
    # Miscellaneous low-level entry points to support initializing a spear.Instance
    #

    def ping(self):
        return self.entry_point_caller.call_on_worker_thread("ping", None)

    def get_id(self):
        return self.entry_point_caller.call_on_worker_thread("get_id", None)

    #
    # Miscellaneous low-level entry points that interact with Unreal globals
    #

    def get_engine(self):
        return self.entry_point_caller.call_on_worker_thread("get_engine", None)

    def is_with_editor(self):
        return self.entry_point_caller.call_on_worker_thread("is_with_editor", None)

    def is_running_commandlet(self):
        return self.entry_point_caller.call_on_worker_thread("is_running_commandlet", None)

    def is_async_loading(self):
        return self.entry_point_caller.call_on_game_thread("is_async_loading", None)

    #
    # Miscellaneous low-level entry points that interact with Unreal singleton structs and are needed in the
    # implementation of spear.Instance. We could implement UCLASSES and UFUNCTIONS to access the low-level
    # Unreal structs, but then we would be limited to accessing them on the game thread. Providing access
    # through these entry points enables access on the worker thread, which simplifies the implmeentation of
    # spear.Instance.
    #

    def get_command_line(self):
        return self.entry_point_caller.call_on_worker_thread("get_command_line", None)

    def request_exit(self, immediate_shutdown):
        self.entry_point_caller.call_on_worker_thread("request_exit", None, immediate_shutdown)
