#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os

def log(*args):
    current_frame = inspect.currentframe()
    print(_log_get_prefix(current_frame) + "".join([str(arg) for arg in args]))

def _log_get_prefix(current_frame):
    return "[SPEAR | " + _get_current_file_abbreviated(current_frame) + ":" + _get_current_line(current_frame) + "] "

def _get_current_file_abbreviated(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return os.path.basename(outer_frames[1].filename)

def _get_current_line(current_frame):
    outer_frames = inspect.getouterframes(current_frame)
    return str(outer_frames[1].lineno)