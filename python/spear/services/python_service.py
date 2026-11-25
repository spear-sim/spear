#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import ast
import json
import spear


class PythonService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, cdo, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            cdo=cdo,
            config=config)

    def initialize(self):
        assert self.cdo.UPythonScriptLibrary.IsPythonAvailable()

        # make it so the spear namespace can be used more easily
        self.execute_statement(statement="import numpy as np")
        self.execute_statement(statement="import spear")
        self.execute_statement(statement="import unreal")

    #
    # Functions for executing long-running editor scripts that span multiple frames.
    #

    # Execute file with command-line arguments across multiple frames. Note that inputs and outputs are not
    # supported because the file might execute in the Private scope.
    def execute_file_across_frames(self, file, args_string, execution_scope="Private"):
        python_command = f"{file} {args_string}"
        assert self._get_file_from_python_command(python_command=python_command) == file

        engine_service = self.entry_point_caller.engine_service

        with engine_service.begin_frame():
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=python_command, execution_mode="ExecuteFile", file_execution_scope=execution_scope)
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
            self.execute_statement(statement=f"spear.editor.ScriptRunner.destroy_token(token='{token}')")
        with engine_service.end_frame():
            pass

        if exception_text is not None:
            spear.log("EXCEPTION (Unreal Editor): ", exception_text)
            raise RuntimeError(exception_text)

    def execute_file_across_frames_in_editor_script(self, file, args_string, execution_scope="Private"):
        python_command = f"{file} {args_string}"
        assert self._get_file_from_python_command(python_command=python_command) == file

        engine_service = self.entry_point_caller.engine_service

        with engine_service.begin_frame():
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=python_command, execution_mode="ExecuteFile", file_execution_scope=execution_scope)
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
            self.execute_statement(statement=f"spear.editor.ScriptRunner.destroy_token(token='{token}')")
        yield
        with engine_service.end_frame():
            pass
        yield

        if exception_text is not None:
            spear.log("EXCEPTION (Unreal Editor): ", exception_text)
            raise RuntimeError(exception_text)

    # Execute a string with optional strongly typed inputs and outputs across multiple frames. Literal code
    # always executes in the Public scope, so inputs and outputs are always supported.
    def execute_string_across_frames(self, string, inputs=None, outputs=None):
        assert self._get_file_from_python_command(python_command=string) is None

        engine_service = self.entry_point_caller.engine_service

        with engine_service.begin_frame():
            if inputs is not None:
                for name, value in inputs.items():
                    self.set(name=name, value=value)
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", file_execution_scope="Public")
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
            self.execute_statement(statement=f"spear.editor.ScriptRunner.destroy_token(token='{token}')")
            if outputs is None:
                result = None
            else:
                # resolved_outputs for consistency with the convert_func closure in execute_string
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

    def execute_string_across_frames_in_editor_script(self, string, inputs=None, outputs=None):
        assert self._get_file_from_python_command(python_command=string) is None

        engine_service = self.entry_point_caller.engine_service

        with engine_service.begin_frame():
            if inputs is not None:
                for name, value in inputs.items():
                    self.set(name=name, value=value)
            token = self.evaluate_expression(expression="spear.editor.ScriptRunner.create_token()")
            self.execute_statement(statement=f"spear.editor.ScriptRunner.set_pending_token(token='{token}')")
            self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", file_execution_scope="Public")
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
            self.execute_statement(statement=f"spear.editor.ScriptRunner.destroy_token(token='{token}')")
            if outputs is None:
                result = None
            else:
                # resolved_outputs for consistency with the convert_func closure in execute_string
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
            as_handle=as_handle, as_unreal_struct=as_unreal_struct, as_unreal_class=as_unreal_class, as_unreal_object=as_unreal_object, with_sp_funcs=with_sp_funcs)

    def set(self, name, value):
        return self.execute_statement(
            statement=f"{name} = {spear.to_script_expr(obj=value)}")

    #
    # Execute files, strings, statements, and expressions. Unreal only applies FileExecutionScope when
    # executing a file, so only the execute_file* functions expose it. Strings, statements, and expressions
    # always execute in the Public scope. Note that Unreal treats any string containing ".py" followed by
    # whitespace (or the end of the string) as a file, so avoid this pattern in literal code passed to the
    # execute_string* functions.
    #

    # Execute file with command-line arguments. Note that inputs and outputs are not supported because the
    # file might execute in the Private scope. If called synchronously, returns nothing. If called
    # asynchronously, returns a spear.Future.
    def execute_file(self, file, args_string, execution_scope="Private"):
        python_command = f"{file} {args_string}"
        assert self._get_file_from_python_command(python_command=python_command) == file

        result = self.execute_python_command_ex(python_command=python_command, execution_mode="ExecuteFile", file_execution_scope=execution_scope)

        def convert_func(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['Type'].upper()} (Unreal Editor): {log_output['Output'].strip()}")
            if not result["ReturnValue"]:
                spear.log("ERROR (Unreal Editor): ", result["CommandResult"])
                raise RuntimeError(result["CommandResult"])

        return self._to_result(result=result, convert_func=convert_func)

    # Execute a string with optional strongly typed inputs and outputs. If called synchronously, returns
    # user-specified outputs in a dict. If called asynchronously, returns a spear.Future.
    def execute_string(self, string, inputs=None, outputs=None):
        assert self._get_file_from_python_command(python_command=string) is None

        if inputs is not None:
            for name, value in inputs.items():
                self.set(name=name, value=value)

        result = self.execute_python_command_ex(python_command=string, execution_mode="ExecuteFile", file_execution_scope="Public")

        def convert_func(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['Type'].upper()} (Unreal Editor): {log_output['Output'].strip()}")
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

        return self._to_result(result=result, convert_func=convert_func)

    # Execute a single statement. If called synchronously, returns nothing. If called asynchronously, returns
    # a spear.Future.
    def execute_statement(self, statement):
        python_command = statement
        result = self.execute_python_command_ex(python_command=python_command, execution_mode="ExecuteStatement")

        def convert_func(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['Type'].upper()} (Unreal Editor): {log_output['Output'].strip()}")
            if not result["ReturnValue"]:
                spear.log("ERROR (Unreal Editor): ", result["CommandResult"])
                raise RuntimeError(result["CommandResult"])

        return self._to_result(result=result, convert_func=convert_func)

    # Evaluate a single expression. If called synchronously, returns a strongly typed result. If called
    # asynchronously, returns a spear.Future.
    def evaluate_expression(self, expression, as_handle=None, as_unreal_struct=None, as_unreal_class=None, as_unreal_object=None, with_sp_funcs=None):
        python_command = f"spear.editor.to_script_result(obj={expression})"
        result = self.execute_python_command_ex(python_command=python_command, execution_mode="EvaluateStatement")

        def convert_func(result):
            for log_output in result["LogOutput"]:
                spear.log(f"{log_output['Type'].upper()} (Unreal Editor): {log_output['Output'].strip()}")
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

        return self._to_result(result=result, convert_func=convert_func)

    # Exposes the low-level UFUNCTION used to interact with Editor Python. Most users will not need to call
    # this function directly.
    def execute_python_command_ex(self, python_command, execution_mode="ExecuteFile", file_execution_scope="Private"):
        return self.cdo.UPythonScriptLibrary.ExecutePythonCommandEx(PythonCommand=python_command, ExecutionMode=execution_mode, FileExecutionScope=file_execution_scope, as_dict=True)

    # PythonService calls a cdo UFUNCTION rather than an entry point, so it can't pass convert_func to an
    # EntryPointCaller. Instead, we apply convert_func to the result directly, or, if the cdo was called via
    # call_async, we apply convert_func when Future.get() is called via Future.apply_last(...).
    def _to_result(self, result, convert_func):
        if isinstance(result, dict):
            return convert_func(result=result)
        elif isinstance(result, spear.Future):
            result.apply_last(convert_func=convert_func)
            return result
        else:
            assert False

    # Mirrors TryExtractPathnameAndCommand in PythonScriptPlugin.cpp. Returns the file that Unreal will
    # execute if it interprets python_command as a file, or None if it will interpret it as literal code.
    def _get_file_from_python_command(self, python_command):
        pos = python_command.find(".py")
        if pos == -1:
            return None
        end = pos + len(".py")
        file = python_command[:end]
        if file.startswith('"'):
            file = file[1:]
            if end == len(python_command):
                return None
            for c in python_command[end:]:
                if c == '"':
                    break
                elif c.isspace():
                    continue
                else:
                    return None
        elif end < len(python_command) and not python_command[end].isspace():
            return None
        return file
