#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os

def log(message):
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + message)

def log_current_function():
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + _get_current_function_abbreviated(current_frame))

def log_no_prefix(message):
    print(message)

def log_get_prefix():
    current_frame = inspect.currentframe()
    return _log_get_prefix(current_frame)

def _log_get_prefix(current_frame):
    return "[SPEAR | " + _get_current_file_abbreviated(current_frame) + "] "

def _get_current_file_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].filename)

def _get_current_function_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].function)
