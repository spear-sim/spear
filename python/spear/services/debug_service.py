#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

class DebugService(spear.Service):
    def __init__(self, entry_point_caller):
        super().__init__(entry_point_caller=entry_point_caller)

    def assert_false_on_game_thread(self):
        self.entry_point_caller.call_on_game_thread("assert_false_on_game_thread", None)

    def assert_false_on_worker_thread(self):
        self.entry_point_caller.call_on_worker_thread("assert_false_on_worker_thread", None)

    def throw_exception_on_game_thread(self):
        self.entry_point_caller.call_on_game_thread("throw_exception_on_game_thread", None)

    def throw_exception_on_worker_thread(self):
        self.entry_point_caller.call_on_worker_thread("throw_exception_on_worker_thread", None)
