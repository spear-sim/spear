#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import sys

if spear.__can_import_unreal__:
    import unreal


class EngineService():
    def __init__(self, client, config):

        self._client = client
        self._config = config

        self._frame_state = None
        self._editor_transaction_state = None
        self._byte_order = None
        self.entry_point_signature_descs = None

    def initialize(self):
        assert self._frame_state is None or self._frame_state == "idle" or self._frame_state == "error"
        assert self._editor_transaction_state is None or self._editor_transaction_state == "idle" or self._editor_transaction_state == "error"

        self._frame_state = "idle"
        self._editor_transaction_state = "idle"

        # explicitly initialize before calling begin_frame() for the first time
        self._client.call("engine_service.call_sync_on_worker_thread.initialize")

        # cache data that will be constant for the life of the server

        if self.entry_point_signature_descs is None:
            self.entry_point_signature_descs = self._client.get_entry_point_signature_descs()

        if self._byte_order is None:
            byte_order = self._client.call("engine_globals_service.call_sync_on_worker_thread.get_byte_order")
            if byte_order == sys.byteorder:
                self._byte_order = "native"
            else:
                self._byte_order = byte_order

    def terminate(self):
        assert self._frame_state is not None
        self._frame_state = None
        self._editor_transaction_state = None
        self._client.call("engine_service.call_sync_on_worker_thread.terminate")

    #
    # Helper functions to support initializing a spear.Instance
    #

    def get_byte_order(self):
        assert self._byte_order is not None
        return self._byte_order

    #
    # These context managers are intended as exception-safe wrappers for managing the server's frame state.
    #

    @contextlib.contextmanager
    def begin_frame(self):
        if spear.__can_import_unreal__:
            assert self._frame_state == "idle" or self._frame_state == "request_begin_next_frame"
            self._frame_state = "executing_begin_frame"
            yield
            assert self._frame_state == "executing_begin_frame"
            self._frame_state = "executing_frame"
            return

        if spear.__can_import_spear_ext__:
            if self._frame_state in ["idle", "request_begin_next_frame"]:
                pass
            elif self._frame_state in ("executing_begin_frame", "request_begin_frame"):
                spear.log("Unexpected frame state: ", self._frame_state)
                spear.log("ERROR: Attempting to exit critical section by calling _execute_frame_impl and _end_frame_impl()...")
                self._execute_frame_impl()
                self._end_frame_impl()
                assert False
            elif self._frame_state in ("executing_frame", "executing_end_frame", "request_end_frame"):
                spear.log("Unexpected frame state: ", self._frame_state)
                spear.log("ERROR: Attempting to exit critical section by calling _execute_frame_impl and _end_frame_impl()...")
                self._end_frame_impl()
                assert False
            elif self._frame_state == "error":
                spear.log("Unexpected frame state: ", self._frame_state)
                assert False
            else:
                spear.log(f"ERROR: Don't know how to recover from frame state '{self._frame_state}'")
                assert False

        success = False

        # if we're currently in the idle state, we need to call begin_frame_impl()
        if self._frame_state == "idle":
            self._frame_state = "request_begin_frame"

            # try calling begin_frame_impl()
            try:
                success = self._begin_frame_impl()
            except Exception as e:
                spear.log("Exception: ", e)
                spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
                self._frame_state = "error"
                raise

            if not success:
                spear.log("ERROR: Server error state detected when calling _begin_frame_impl(), but we do not appear to be in a critical section.")
                self._frame_state = "error"
                assert False

        # if we're currently in the request_begin_next_frame state, it means that the user previously called
        # end_frame(single_step=True), in which case we have effectively already called begin_frame_impl() as
        # part of our previous call to end_frame_impl_single_step(), so we do not call it again here
        elif self._frame_state == "request_begin_next_frame":
            self._frame_state = "request_begin_frame"

        else:
            assert False

        # try executing pre-frame work
        try:
            assert self._frame_state == "request_begin_frame"
            self._frame_state = "executing_begin_frame"
            yield
            assert self._frame_state == "executing_begin_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling _execute_frame_impl() and _end_frame_impl()...")
            self._execute_frame_impl() # ignore return value because we already know we need to call _execute_frame_impl() and _end_frame_impl()
            self._end_frame_impl()
            self._frame_state = "error"
            raise

        success = False

        # try calling execute_frame_impl()
        try:
            success = self._execute_frame_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We are in a critical section, but we don't know how to get out of it from here. Attempting to call _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise

        if not success:
            spear.log("ERROR: Server error state detected when calling _execute_frame_impl(). Attempting to exit critical section by calling _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            assert False

        self._frame_state = "executing_frame"

    @contextlib.contextmanager
    def end_frame(self, single_step=False):
        if spear.__can_import_unreal__:
            assert self._frame_state == "executing_frame"
            self._frame_state = "executing_end_frame"
            yield
            assert self._frame_state == "executing_end_frame"
            if single_step:
                self._frame_state = "request_begin_next_frame"
            else:
                self._frame_state = "idle"
            return

        # try executing post-frame work
        try:
            assert self._frame_state == "executing_frame"
            self._frame_state = "executing_end_frame"
            yield
            assert self._frame_state == "executing_end_frame"
            self._frame_state = "request_end_frame"
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: Attempting to exit critical section by calling _end_frame_impl()...")
            self._end_frame_impl()
            self._frame_state = "error"
            raise

        success = False

        # try calling end_frame_impl()
        try:
            success = self._end_frame_impl(single_step=single_step)
        except Exception as e:
            spear.log("Exception: ", e)
            spear.log("ERROR: We might or might not be in a critical section, but we don't know how to get out of it from here. Giving up...")
            self._frame_state = "error"
            raise

        assert self._frame_state == "request_end_frame"

        if not success:
            spear.log("ERROR: Server error state detected when calling _end_frame_impl_single_step(), but we do not appear to be in a critical section.")
            self._frame_state = "error"
            assert False

        if single_step:
            self._frame_state = "request_begin_next_frame"
        else:
            self._frame_state = "idle"

    @contextlib.contextmanager
    def editor_transaction(self):
        assert spear.__can_import_unreal__
        assert self._editor_transaction_state == "idle"
        self._editor_transaction_state = "request_editor_transaction"

        success = False

        try:
            success = self._begin_editor_transaction_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            self._editor_transaction_state = "error"
            raise

        if not success:
            spear.log("ERROR: Server error state detected when calling _begin_editor_transaction_impl().")
            self._editor_transaction_state = "error"
            assert False

        try:
            assert self._editor_transaction_state == "request_editor_transaction"
            self._editor_transaction_state = "executing_editor_transaction"
            yield
            assert self._editor_transaction_state == "executing_editor_transaction"
            self._editor_transaction_state = "executing_editor_post_transaction"
        except Exception as e:
            spear.log("Exception: ", e)
            self._editor_transaction_state = "error"
            raise

        success = False

        try:
            success = self._end_editor_transaction_impl()
        except Exception as e:
            spear.log("Exception: ", e)
            self._editor_transaction_state = "error"
            raise

        assert self._editor_transaction_state == "executing_editor_post_transaction"

        if not success:
            spear.log("ERROR: Server error state detected when calling _end_editor_transaction_impl().")
            self._editor_transaction_state = "error"
            assert False

        unreal.SpEngineService.execute_editor_post_transaction()

        assert self._editor_transaction_state == "executing_editor_post_transaction"
        self._editor_transaction_state = "idle"


    #
    # Helper functions for managing the server's frame state.
    #

    def _begin_frame_impl(self):
        return self.call_on_worker_thread("engine_service.call_sync_on_worker_thread.begin_frame")

    def _execute_frame_impl(self):
        return self.call_on_worker_thread("engine_service.call_sync_on_worker_thread.execute_frame")

    def _end_frame_impl(self, single_step=False):
        return self.call_on_worker_thread("engine_service.call_sync_on_worker_thread.end_frame", single_step)

    def _begin_editor_transaction_impl(self):
        return self.call_on_worker_thread("engine_service.call_sync_on_worker_thread.begin_editor_transaction")

    def _end_editor_transaction_impl(self):
        return self.call_on_worker_thread("engine_service.call_sync_on_worker_thread.end_editor_transaction")

    def flush(self):
        self.call_on_game_thread("engine_service.call_sync_on_game_thread.flush")

    #
    # Functions for calling entry points on the server.
    #

    def call_on_worker_thread(self, func_name, *args):
        self._validate_frame_state_for_worker_thread_work()
        if spear.__can_import_unreal__:
            self._validate_editor_transaction_state_for_worker_thread_work()
        return self._call_impl(func_name, *args)

    def call_on_game_thread(self, func_name, *args):
        self._validate_frame_state_for_game_thread_work()
        if spear.__can_import_unreal__:
            self._validate_editor_transaction_state_for_game_thread_work()
        return self._call_impl(func_name, *args)

    def get_future_result_from_game_thread(self, future, return_as):
        self._validate_frame_state_for_worker_thread_work()
        if spear.__can_import_unreal__:
            self._validate_editor_transaction_state_for_worker_thread_work()
        return self._call_impl(f"engine_service.get_future_result_from_game_thread_as_{return_as}", future)

    def _call_impl(self, func_name, *args):
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            return_as = self.entry_point_signature_descs[func_name].type_names[0]
            spear.log(f"Calling: {func_name} : {args} -> {return_as}")
        return_value = self._client.call(func_name, *args)
        if self._config.SPEAR.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO:
            spear.log(f"Obtained return value: {return_value}")
        return return_value

    #
    # Helper functions for calling entry points on the server
    #

    def _validate_frame_state_for_worker_thread_work(self):
        if self._frame_state not in ["idle", "request_begin_frame", "executing_begin_frame", "executing_frame", "executing_end_frame", "request_end_frame"]:
            spear.log("ERROR: Unexpected frame state: ", self._frame_state)
            assert False

    def _validate_frame_state_for_game_thread_work(self):
        if self._frame_state not in ["executing_begin_frame", "executing_end_frame"]:
            spear.log('ERROR: Entry points that execute on the game thread are only allowed in "with begin_frame()" or "with end_frame()" code blocks.')
            spear.log(f'ERROR: self._frame_state == "{self._frame_state}"')

            if self._frame_state == "executing_frame":
                spear.log("ERROR: Attempting to exit critical section by calling _end_frame_impl()...")
                self._end_frame_impl()
                self._frame_state = "error"

            assert False

    def _validate_editor_transaction_state_for_worker_thread_work(self):
        if self._editor_transaction_state not in ["idle", "request_editor_transaction", "executing_editor_transaction", "executing_editor_post_transaction"]:
            spear.log("ERROR: Unexpected editor transaction state: ", self._editor_transaction_state)
            assert False

    def _validate_editor_transaction_state_for_game_thread_work(self):
        if self._editor_transaction_state not in ["executing_editor_transaction"]:
            spear.log('ERROR: Entry points that execute on the game thread are only allowed in "with editor_transaction()" code blocks.')
            spear.log(f'ERROR: self._editor_transaction_state == "{self._editor_transaction_state}"')
            assert False
