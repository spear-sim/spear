#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class EntryPointCaller():
    def __init__(self, service_name, engine_service):
        self.service_name = service_name
        self.engine_service = engine_service

    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        return_value = self.engine_service.call_on_game_thread(return_as, long_func_name, *args)
        if return_value is not None and convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value

    def call_on_worker_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        return_value = self.engine_service.call_on_worker_thread(return_as, long_func_name, *args)
        if return_value is not None and convert_func is not None:
            return convert_func(return_value)
        else:
            return return_value


class CallAsyncEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        future = self.engine_service.call_async_on_game_thread(long_func_name, *args)
        return spear.utils.func_utils.Future(
            future=future, return_as=return_as, get_future_result_func=self.engine_service.get_future_result, convert_func=convert_func)


class SendAsyncEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        self.engine_service.send_async_on_game_thread(long_func_name, *args)


class CallAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        future = self.engine_service.call_async_fast_on_game_thread(long_func_name, *args)
        return spear.utils.func_utils.Future(
            future=future, return_as=return_as, get_future_result_func=self.engine_service.get_future_result_fast, convert_func=convert_func)


class SendAsyncFastEntryPointCaller(EntryPointCaller):
    def call_on_game_thread(self, return_as, func_name, convert_func, *args):
        long_func_name = self.service_name + "." + func_name
        self.engine_service.send_async_fast_on_game_thread(long_func_name, *args)
