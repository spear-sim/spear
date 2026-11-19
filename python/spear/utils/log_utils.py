#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os

_default_log_enabled = True
_log_funcs = []

def get_default_log_enabled():
    return _default_log_enabled

def set_default_log_enabled(enabled):
    global _default_log_enabled
    old_enabled = _default_log_enabled
    _default_log_enabled = enabled
    return old_enabled

def register_log_func(func):
    _log_funcs.append(func)

def unregister_log_func(func):
    _log_funcs.remove(func)

def log(*args):
    caller_frame = inspect.currentframe().f_back
    message = _log_get_prefix(caller_frame=caller_frame) + "".join([str(arg) for arg in args])
    if _default_log_enabled:
        print(message, flush=True)
    for func in _log_funcs:
        func(message)

def log_current_function(prefix=""):
    caller_frame = inspect.currentframe().f_back
    message = _log_get_prefix(caller_frame=caller_frame) + prefix + caller_frame.f_code.co_qualname
    if _default_log_enabled:
        print(message, flush=True)
    for func in _log_funcs:
        func(message)

def log_no_prefix(*args):
    message = "".join([str(arg) for arg in args])
    if _default_log_enabled:
        print(message, flush=True)
    for func in _log_funcs:
        func(message)

def log_get_prefix():
    caller_frame = inspect.currentframe().f_back
    return _log_get_prefix(caller_frame=caller_frame)

def _log_get_prefix(caller_frame):
    return f"[SPEAR | {os.path.basename(caller_frame.f_code.co_filename)}:{caller_frame.f_lineno:04}] "
