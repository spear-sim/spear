#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import ast
import json
import spear


class PythonService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, config, parent_service=None, create_children_services=True):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        # do this after initializing local state
        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config,
            parent_service=parent_service,
            create_children_services=create_children_services)

        self._python_script_library = None

    def create_child_service(self, entry_point_caller, sp_func_service=None, unreal_service=None, config=None):
        assert self.is_top_level_service()
        return PythonService(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config,
            parent_service=self,
            create_children_services=False)

    def initialize(self):
        assert self.is_top_level_service()

        self._python_script_library = spear.UnrealObject(
            unreal_service=self.unreal_service,
            sp_func_service=self.sp_func_service,
            config=self.config,
            uclass="UPythonScriptLibrary")

        assert self._python_script_library.IsPythonAvailable()

        # make it so the spear namespace can be used more easily
        self.execute_statement(statement="import numpy as np", execution_scope="Public")
        self.execute_statement(statement="import spear", execution_scope="Public")
        self.execute_statement(statement="import unreal", execution_scope="Public")

        # wire up async variants so child services route through the appropriate async UnrealObject
        self.call_async._python_script_library = self._python_script_library.call_async
        self.send_async._python_script_library = self._python_script_library.send_async
        self.call_async_fast._python_script_library = self._python_script_library.call_async_fast
        self.send_async_fast._python_script_library = self._python_script_library.send_async_fast

    #
    # Functions for executing long-running editor scripts that span multiple frames.
    #

    def execute_file_across_frames(self, file, args_string, execution_scope="Private"):
        return self.execute_string_across_frames(string=f"{file} {args_string}", execution_scope=execution_scope)

    def execute_file_across_frames_in_editor_script(self, file, args_string, execution_scope="Private"):
        return (yield from self.execute_string_across_frames_in_editor_script(string=f"{file} {args_string}", execution_scope=execution_scope))

    def execute_string_across_frames(self, string, execution_scope="Private", inputs=None, outputs=None):
        engine_service = self.entry_point_caller.engine_service

        if execution_scope != "Public":
            assert inputs is None
            assert outputs is None

        with engine_service.begin_frame():
            if inputs is not None:
                for name, value in inputs.items():
                    self.set(name=name, value=value)
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", execution_scope=execution_scope)
            was_synchronous = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.get_pending_token() == '{token}'")
            if was_synchronous:
                self.execute_statement(statement="spear.editor.ScriptRunner.clear_pending_token()")
        with engine_service.end_frame():
            pass

        if not was_synchronous:
            completed = False
            while not completed:
                with engine_service.begin_frame():
                    for message in self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_log_messages(token='{token}')"):
                        spear.log("INFO (Unreal Editor): ", message)
                    completed = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.is_complete(token='{token}')")
                with engine_service.end_frame():
                    pass

        with engine_service.begin_frame():
            for message in self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_log_messages(token='{token}')"):
                spear.log("INFO (Unreal Editor): ", message)
            exception_text = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_exception(token='{token}')")
            if outputs is None:
                result = None
            else:
                # resolved_outputs for consistency with the process_result closure in execute_string
                if isinstance(outputs, list):
                    resolved_outputs = { name: {} for name in outputs }
                elif isinstance(outputs, dict):
                    resolved_outputs = { name: (args if args is not None else {}) for name, args in outputs.items() }
                else:
                    assert False
                result = { name: self.get(name=name, **args) for name, args in resolved_outputs.items() }
        with engine_service.end_frame():
            pass

        if exception_text is not None:
            spear.log("EXCEPTION (Unreal Editor): ", exception_text)
            raise RuntimeError(exception_text)

        return result

    def execute_string_across_frames_in_editor_script(self, string, execution_scope="Private", inputs=None, outputs=None):
        engine_service = self.entry_point_caller.engine_service

        if execution_scope != "Public":
            assert inputs is None
            assert outputs is None

        with engine_service.begin_frame():
            if inputs is not None:
                for name, value in inputs.items():
                    self.set(name=name, value=value)
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", execution_scope=execution_scope)
            was_synchronous = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.get_pending_token() == '{token}'")
            if was_synchronous:
                self.execute_statement(statement="spear.editor.ScriptRunner.clear_pending_token()")
        yield
        with engine_service.end_frame():
            pass
        yield

        if not was_synchronous:
            completed = False
            while not completed:
                with engine_service.begin_frame():
                    for message in self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_log_messages(token='{token}')"):
                        spear.log("INFO (Unreal Editor): ", message)
                    completed = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.is_complete(token='{token}')")
                yield
                with engine_service.end_frame():
                    pass
                yield

        with engine_service.begin_frame():
            for message in self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_log_messages(token='{token}')"):
                spear.log("INFO (Unreal Editor): ", message)
            exception_text = self.evaluate_expression(expression=f"spear.editor.ScriptRunner.clear_exception(token='{token}')")
            if outputs is None:
                result = None
            else:
                # resolved_outputs for consistency with the process_result closure in execute_string
                if isinstance(outputs, list):
                    resolved_outputs = { name: {} for name in outputs }
                elif isinstance(outputs, dict):
                    resolved_outputs = { name: (args if args is not None else {}) for name, args in outputs.items() }
                else:
                    assert False
                result = { name: self.get(name=name, **args) for name, args in resolved_outputs.items() }
        yield
        with engine_service.end_frame():
            pass
        yield

        if exception_text is not None:
            spear.log("EXCEPTION (Unreal Editor): ", exception_text)
            raise RuntimeError(exception_text)

        return result

    #
    # Get and set are strongly typed and execute in the Public scope
    #

    def get(self, name, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        return self.evaluate_expression(
            expression=name,
            execution_scope="Public",
            as_handle=as_handle, as_unreal_struct=as_unreal_struct, as_unreal_class=as_unreal_class, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def set(self, name, value):
        return self.execute_statement(
            statement=f"{name} = {spear.to_script_expr(obj=value)}",
            execution_scope="Public")

    #
    # All other high-level methods execute in the Private scope by default.
    #

    # Execute file with command-line arguments. If called synchronously, returns nothing. If called
    # asynchronously, returns a spear.Future.
    def execute_file(self, file, args_string, execution_scope="Private"):
        return self.execute_string(string=f"{file} {args_string}", execution_scope=execution_scope)

    # Execute a string with optional strongly typed inputs and outputs. Note that inputs and outputs are only
    # allowed if the execution scope is Public. If called synchronously, returns user-specified outputs in a
    # dict. If called asynchronously, returns a spear.Future.
    def execute_string(self, string, execution_scope="Private", inputs=None, outputs=None):

        if execution_scope != "Public":
            assert inputs is None
            assert outputs is None

        if inputs is not None:
            for name, value in inputs.items():
                self.set(name=name, value=value)

        result = self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", execution_scope=execution_scope)

        def process_result(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['type'].upper()} (Unreal Editor): {log_output['output'].strip()}")
            if not result["ReturnValue"]:
                spear.log("ERROR (Unreal Editor): ", result["CommandResult"])
                raise RuntimeError(result["CommandResult"])

            if outputs is None:
                return None
            else:
                # resolved_outputs avoids overwriting the outputs closure variable which would cause UnboundLocalError
                if isinstance(outputs, list):
                    resolved_outputs = { name: {} for name in outputs }
                elif isinstance(outputs, dict):
                    resolved_outputs = { name: (args if args is not None else {}) for name, args in outputs.items() }
                else:
                    assert False
                return { name: self.get(name=name, **args) for name, args in resolved_outputs.items() }

        return self._get_result(result=result, process_result_func=process_result)

    # Execute a single statement. If called synchronously, returns nothing. If called asynchronously, returns
    # a spear.Future.
    def execute_statement(self, statement, execution_scope="Private"):
        python_command = statement
        result = self.execute_python_command_ex(python_command=python_command, execution_mode="ExecuteStatement", execution_scope=execution_scope)

        def process_result(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['type'].upper()} (Unreal Editor): {log_output['output'].strip()}")
            if not result["ReturnValue"]:
                spear.log("ERROR (Unreal Editor): ", result["CommandResult"])
                raise RuntimeError(result["CommandResult"])

        return self._get_result(result=result, process_result_func=process_result)

    # Evaluate a single expression. If called synchronously, returns a strongly typed result. If called
    # asynchronously, returns a spear.Future.
    def evaluate_expression(self, expression, execution_scope="Private", as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        python_command = f"spear.editor.to_script_result(obj={expression})"
        result = self.execute_python_command_ex(python_command=python_command, execution_mode="EvaluateStatement", execution_scope=execution_scope)

        def process_result(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['type'].upper()} (Unreal Editor): {log_output['output'].strip()}")
            if result["ReturnValue"]:
                script_result = json.loads(ast.literal_eval(result["CommandResult"])) # CommandResult has extra single quotes so we remove using ast.literal_eval
                result = spear.from_script_result(
                    script_result=script_result,
                    unreal_service=self.unreal_service,
                    sp_func_service=self.sp_func_service,
                    config=self.config,
                    as_handle=as_handle, as_unreal_struct=as_unreal_struct, as_unreal_class=as_unreal_class, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)
                return result
            else:
                spear.log("ERROR (Unreal Editor): ", result["CommandResult"])
                raise RuntimeError(result["CommandResult"])

        return self._get_result(result=result, process_result_func=process_result)

    # Exposes the low-level UFUNCTION used to interact with Editor Python. Most users will not need to call
    # this function directly. 
    def execute_python_command_ex(self, python_command, execution_mode="ExecuteFile", execution_scope="Private"):
        return self._python_script_library.ExecutePythonCommandEx(PythonCommand=python_command, ExecutionMode=execution_mode, FileExecutionScope=execution_scope, as_dict=True)

    def _get_result(self, result, process_result_func):
        if isinstance(result, dict):
            return process_result_func(result)
        elif isinstance(result, spear.Future):
            inner_convert_func = result.convert_func
            def convert_func(o):
                if inner_convert_func is not None:
                    o = inner_convert_func(o)
                return process_result_func(o)
            result.convert_func = convert_func
            return result
        else:
            assert False
