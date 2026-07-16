#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class ConsoleService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

    #
    # Get console variable value
    #

    def get_as_bool(self, name):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.get_console_variable_value_as_bool(cvar=cvar)

    def get_as_int(self, name):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.get_console_variable_value_as_int(cvar=cvar)

    def get_as_float(self, name):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.get_console_variable_value_as_float(cvar=cvar)

    def get_as_string(self, name):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.get_console_variable_value_as_string(cvar=cvar)

    #
    # Set console variable value
    #

    def set(self, name, value, set_by_flags=None):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.set_console_variable_value(cvar=cvar, value=value, set_by_flags=set_by_flags)

    #
    # Execute console command
    #

    def execute(self, command):
        return self.unreal_service.execute_console_command(command=command)
