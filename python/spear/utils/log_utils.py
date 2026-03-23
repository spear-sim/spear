#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os

_log_funcs = []

def register_log_func(func):
    _log_funcs.append(func)

def unregister_log_func(func):
    _log_funcs.remove(func)

def log(*args):
    current_frame = inspect.currentframe()
    message = _log_get_prefix(current_frame) + "".join([str(arg) for arg in args])
    print(message)
    for func in _log_funcs:
        func(message)

def log_current_function(prefix=""):
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + prefix + _get_current_function_abbreviated(current_frame))

def log_no_prefix(*args):
    print("".join([str(arg) for arg in args]))

def log_get_prefix():
    current_frame = inspect.currentframe()
    return _log_get_prefix(current_frame)

def _log_get_prefix(current_frame):
    return f"[SPEAR | {_get_current_file_abbreviated(current_frame)}:{_get_current_line(current_frame)}] "

def _get_current_file_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].filename)

def _get_current_line(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return f"{outer_frames[1].lineno:04}"

def _get_current_function_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return outer_frames[1].function
