#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import inspect
import os
import sys

_RESET = "\033[0m"
_COLORS = {
    "black":   "30",
    "red":     "31",
    "green":   "32",
    "yellow":  "33",
    "blue":    "34",
    "magenta": "35",
    "cyan":    "36",
    "white":   "37",
}

_default_log_enabled = True
_log_funcs = []
_supports_color_cached = None

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

# supports_color() follows the NO_COLOR/FORCE_COLOR conventions (https://no-color.org/) on top of the standard
# library's own notion of whether stdout is a terminal (to avoid dumping escape codes to files for example)
def supports_color():
    global _supports_color_cached
    if _supports_color_cached is not None:
        return _supports_color_cached
    _supports_color_cached = _supports_color()
    return _supports_color_cached

def _supports_color():
    if os.environ.get("NO_COLOR") is not None:
        return False
    if os.environ.get("FORCE_COLOR") is not None:
        return True
    return sys.stdout.isatty()

def colorize(text, color, bold=False):
    if not supports_color():
        return text
    prefix = "1;" if bold else ""
    return f"\033[{prefix}{_COLORS[color]}m{text}{_RESET}"

def colorize_message_category(message):
    message_lower = message.lower()
    if "error" in message_lower or "exception" in message_lower:
        print(colorize(text=message, color="red", bold=True))
    elif "warning" in message_lower:
        print(colorize(text=message, color="yellow"))
    else:
        print(message)

def log(*args):
    current_frame = inspect.currentframe()
    message = _log_get_prefix(current_frame) + "".join([str(arg) for arg in args])
    if _default_log_enabled:
        print(colorize_message_category(message))
    for func in _log_funcs:
        func(message)

def log_current_function(prefix=""):
    current_frame = inspect.currentframe()
    message = _log_get_prefix(current_frame) + prefix + _get_current_function_abbreviated(current_frame)
    if _default_log_enabled:
        print(message)
    for func in _log_funcs:
        func(message)

def log_no_prefix(*args):
    message = "".join([str(arg) for arg in args])
    if _default_log_enabled:
        print(message)
    for func in _log_funcs:
        func(message)

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
