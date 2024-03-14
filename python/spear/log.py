#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os


def log(*args):
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + "".join([str(arg) for arg in args]))

def log_current_function():
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + _get_current_function_abbreviated(current_frame))

def log_no_prefix(*args):
    print("".join([str(arg) for arg in args]))

def log_get_prefix():
    current_frame = inspect.currentframe()
    return _log_get_prefix(current_frame)

def _log_get_prefix(current_frame):
    return "[SPEAR | " + _get_current_file_abbreviated(current_frame) + ":" + _get_current_line(current_frame) + "] "

def _get_current_file_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].filename)

def _get_current_line(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return str(outer_frames[1].lineno)

def _get_current_function_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].function)
