#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import numpy as np
import os
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    editor = instance.get_editor()

    inner_script_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "inner_script.py"))
    inner_script_across_frames_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "inner_script_across_frames.py"))
    inner_script_raise_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "inner_script_raise.py"))

    #
    # execute_file_across_frames (called outside begin_frame/end_frame since it manages its own frames)
    #

    # test with a script that uses @spear.editor.script (multi-frame)
    editor.editor_python_service.execute_file_across_frames(file=inner_script_across_frames_file, args_string="--num-yields 3 --scale 4.0", execution_scope="Public")
    with instance.begin_frame():
        result = editor.editor_python_service.get(name="across_frames_result")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result)
        np.testing.assert_array_equal(result, np.array([4.0, 8.0, 12.0]))
    with instance.end_frame():
        pass

    # test with a synchronous script (no @spear.editor.script, should complete immediately)
    editor.editor_python_service.execute_file_across_frames(file=inner_script_file, args_string="--scale 3.0", execution_scope="Public")
    with instance.begin_frame():
        result = editor.editor_python_service.get(name="inner_script_result")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result)
        np.testing.assert_array_equal(result, np.array([3.0, 6.0, 9.0]))
    with instance.end_frame():
        pass

    # test exception propagation
    try:
        editor.editor_python_service.execute_file_across_frames(file=inner_script_raise_file, args_string="")
        assert False # expecting AssertionError from inner_script_raise
    except AssertionError as e:
        spear.log("Caught expected error: ", type(e))

    #
    # execute_string_across_frames with inputs and outputs
    #

    result = editor.editor_python_service.execute_string_across_frames(
        string="eaf_output = eaf_input * 5",
        execution_scope="Public",
        inputs={"eaf_input": 10},
        outputs=["eaf_output"])
    spear.log("Expecting dict:         ", type(result), " ", result)
    assert result["eaf_output"] == 50

    with instance.begin_frame():

        #
        # get and set
        #

        editor.editor_python_service.set(name="test_int", value=42)
        result = editor.editor_python_service.get(name="test_int")
        spear.log("Expecting int:          ", type(result), " ", result)
        assert result == 42

        editor.editor_python_service.set(name="test_array", value=np.array([1.0, 2.0, 3.0]))
        result = editor.editor_python_service.get(name="test_array")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result)
        np.testing.assert_array_equal(result, np.array([1.0, 2.0, 3.0]))

        #
        # execute_file
        #

        editor.editor_python_service.execute_file(file=inner_script_file, args_string="--scale 5.0", execution_scope="Public")
        result = editor.editor_python_service.get(name="inner_script_result")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result)
        np.testing.assert_array_equal(result, np.array([5.0, 10.0, 15.0]))

        #
        # execute_string
        #

        # execute a string with inputs and outputs (list form)
        result = editor.editor_python_service.execute_string(
            string="es_output = es_input * 3",
            execution_scope="Public",
            inputs={"es_input": 7},
            outputs=["es_output"])
        spear.log("Expecting dict:         ", type(result), " ", result)
        assert result["es_output"] == 21

        # execute a string with numpy inputs and outputs (dict form with kwargs)
        result = editor.editor_python_service.execute_string(
            string="es_arr_output = es_arr_input + 1",
            execution_scope="Public",
            inputs={"es_arr_input": np.array([10.0, 20.0, 30.0])},
            outputs={"es_arr_output": None})
        spear.log("Expecting dict:         ", type(result), " ", result)
        np.testing.assert_array_equal(result["es_arr_output"], np.array([11.0, 21.0, 31.0]))

        #
        # execute_statement
        #

        editor.editor_python_service.execute_statement(statement="test_stmt_var = 123")
        result = editor.editor_python_service.evaluate_expression(expression="test_stmt_var", execution_scope="Public")
        spear.log("Expecting int:          ", type(result), " ", result)
        assert result == 123

        #
        # evaluate_expression
        #

        # evaluate primitive types
        result = editor.editor_python_service.evaluate_expression(expression="1 + 1")
        spear.log("Expecting int:          ", type(result), " ", result)
        assert result == 2

        result = editor.editor_python_service.evaluate_expression(expression="3.14")
        spear.log("Expecting float:        ", type(result), " ", result)
        assert result == 3.14

        result = editor.editor_python_service.evaluate_expression(expression="True")
        spear.log("Expecting bool:         ", type(result), " ", result)
        assert result is True

        result = editor.editor_python_service.evaluate_expression(expression="'hello world'")
        spear.log("Expecting str:          ", type(result), " ", result)
        assert result == "hello world"

        result = editor.editor_python_service.evaluate_expression(expression="None")
        spear.log("Expecting None:         ", type(result), " ", result)
        assert result is None

        # evaluate numpy arrays and matrices
        result = editor.editor_python_service.evaluate_expression(expression="np.arange(10, dtype=np.float64).reshape(2, 5)")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result.shape, " ", result.dtype)
        np.testing.assert_array_equal(result, np.arange(10, dtype=np.float64).reshape(2, 5))

        result = editor.editor_python_service.evaluate_expression(expression="np.matrix([[1.0, 2.0], [3.0, 4.0]])")
        spear.log("Expecting np.matrix:    ", type(result), " ", result.shape, " ", result.dtype)
        assert isinstance(result, np.matrix)
        np.testing.assert_array_equal(result, np.matrix([[1.0, 2.0], [3.0, 4.0]]))

        # evaluate recursive lists and dicts containing mixed types
        result = editor.editor_python_service.evaluate_expression(expression="{'array': np.arange(4, dtype=np.float64), 'count': 2}")
        spear.log("Expecting dict:         ", type(result), " ", result)
        np.testing.assert_array_equal(result["array"], np.arange(4, dtype=np.float64))
        assert result["count"] == 2

        result = editor.editor_python_service.evaluate_expression(expression="[1, 'hello', np.array([10, 20, 30])]")
        spear.log("Expecting list:         ", type(result), " ", result)
        assert result[0] == 1
        assert result[1] == "hello"
        np.testing.assert_array_equal(result[2], np.array([10, 20, 30]))

        # evaluate an Unreal struct (e.g., unreal.Vector) and get back a Python dict
        result = editor.editor_python_service.evaluate_expression(expression="unreal.Vector(x=1.0, y=2.0, z=3.0)")
        spear.log("Expecting dict:         ", type(result), " ", result)
        assert isinstance(result, dict)
        assert result["x"] == 1.0
        assert result["y"] == 2.0
        assert result["z"] == 3.0

        # evaluate an Unreal object (e.g., an actor) and get back an UnrealObject
        result = editor.editor_python_service.evaluate_expression(expression="spear.editor.find_actors()[0]")
        spear.log("Expecting UnrealObject: ", type(result), " ", result)
        assert isinstance(result, spear.UnrealObject)

        # evaluate an Unreal object (e.g., an actor) and get back a handle
        result = editor.editor_python_service.evaluate_expression(expression="spear.editor.find_actors()[0]", as_handle=True)
        spear.log("Expecting int:          ", type(result), " ", result)
        assert isinstance(result, int)
        assert result != 0

        # pass a SPEAR-side numpy array into an editor expression using spear.to_script_expr
        arr = np.array([100.0, 200.0, 300.0])
        result = editor.editor_python_service.evaluate_expression(expression=f"{spear.to_script_expr(obj=arr)}*2")
        spear.log("Expecting np.ndarray:   ", type(result), " ", result)
        np.testing.assert_array_equal(result, arr*2)

        # pass a SPEAR-side dict as an Unreal struct using spear.to_script_expr with struct_type
        vec_dict = {"X": 10.0, "Y": 20.0, "Z": 30.0}
        result = editor.editor_python_service.evaluate_expression(expression=f"{spear.to_script_expr(obj=vec_dict, struct_type='unreal.Vector')}.get_editor_property('x')")
        spear.log("Expecting float:        ", type(result), " ", result)
        assert result == 10.0

    with instance.end_frame():
        pass

    spear.log("Done.")
