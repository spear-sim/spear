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

    def get_cvar_as(self, name, as_type):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        if as_type is bool:
            return self.unreal_service.get_console_variable_value_as_bool(cvar=cvar)
        elif as_type is int:
            return self.unreal_service.get_console_variable_value_as_int(cvar=cvar)
        elif as_type is float:
            return self.unreal_service.get_console_variable_value_as_float(cvar=cvar)
        elif as_type is str:
            return self.unreal_service.get_console_variable_value_as_string(cvar=cvar)
        else:
            assert False

    def exists(self, name):
        return self.unreal_service.find_console_variable_by_name(console_variable_name=name) != 0

    #
    # Set console variable value
    #

    def set_cvar(self, name, value, set_by_flags=None, set_with_current_priority=None):
        cvar = self.unreal_service.find_console_variable_by_name(console_variable_name=name)
        assert cvar != 0
        return self.unreal_service.set_console_variable_value(cvar=cvar, value=value, set_by_flags=set_by_flags, set_with_current_priority=set_with_current_priority)

    #
    # Execute console command
    #

    def execute_command(self, command):
        return self.unreal_service.execute_console_command(command=command)

    #
    # Flush output log messages
    #

    def flush_output_log_messages(self):
        return self.unreal_service.flush_output_log_messages()

    #
    # Get message log names and messages
    #

    def get_message_log_names(self):
        return self.unreal_service.get_message_log_names()

    def get_message_log_messages(self, name):
        return self.unreal_service.get_message_log_messages(name=name)
