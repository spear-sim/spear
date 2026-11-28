#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import json
import os
import pathlib
import shutil
import sys

#
# Functions in this file operate on filesystem paths by default. Functions that operate on Unreal content paths (e.g.,
# "/Game/Maps/MyMap") say so in their names.
#

#
# Join one or more path components and return the resulting canonical absolute path, i.e., with symlinks resolved. We
# use this function consistently instead of os.path.join(...), so paths are always absolute, which makes log output
# easier to debug.
#

def realpath(*paths):
    return os.path.realpath(os.path.join(*paths))

#
# Helper functions for paths that handle symlinks robustly. We can't simply call os.path.exists(...) or
# os.path.lexists(...), because a symlink created on Linux, on a drive that is also visible to Windows, might
# not be recognized as a symlink by Windows, and might not even be queryable, e.g., if the file server fails
# to resolve it. In this case, the only way to observe the symlink is that it appears in the listing of its
# parent directory.
#

def path_exists(path):
    if os.path.exists(path) or os.path.islink(path):
        return True

    head, tail = os.path.split(path)
    if os.path.exists(head) or os.path.islink(head):
        if tail == "":
            return os.path.exists(head) or os.path.islink(head)
        else:
            return tail in os.listdir(head)
    else:
        return False

def remove_path(path):
    if not path_exists(path=path):
        return

    if os.path.islink(path):
        os.unlink(path)
    elif os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    else:
        # If we get here, then path_exists(...) found path in the listing of its parent directory, but Python
        # can't classify it, e.g., a symlink created on Linux and viewed on Windows. In this case, try to use
        # the command-line (we don't attempt to use subprocess.run() because it returns an error when
        # attempting to remove broken symlinks).
        if sys.platform == "win32":
            rm_cmd = "del"
        elif sys.platform in ["darwin", "linux"]:
            rm_cmd = "rm"
        else:
            assert False
        cmd = f'{rm_cmd} "{path}"'
        cmd_result = os.system(cmd)
        assert cmd_result == 0

    assert not path_exists(path=path)

#
# Functions for getting a physical filesystem path from a logical content path.
#

def get_plugin_dirs(unreal_project_dir, unreal_engine_dir, follow_symlinks=False):

    if follow_symlinks:
        path_func = realpath
    else:
        path_func = lambda path: path

    plugins_dirs = [
        os.path.join(unreal_project_dir, "Plugins"),
        os.path.join(unreal_engine_dir, "Engine", "Plugins")]

    uprojects = glob.glob(path_func(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    with open(uproject) as f:
        uproject_dict = json.load(f)
    if "AdditionalPluginDirectories" in uproject_dict:
        for additional_plugins_dir in uproject_dict["AdditionalPluginDirectories"]:
            if os.path.isabs(additional_plugins_dir):
                plugins_dirs.append(path_func(additional_plugins_dir))
            else:
                plugins_dirs.append(path_func(os.path.join(unreal_project_dir, additional_plugins_dir)))

    # Unreal names a plugin after its .uplugin file, not after the directory containing it, and plugins can be
    # nested arbitrarily deeply inside a plugins directory (e.g., Engine/Plugins/Runtime), so we search
    # recursively for .uplugin files.
    plugin_dirs = {}
    for plugins_dir in plugins_dirs:
        for uplugin in glob.glob(path_func(os.path.join(plugins_dir, "**", "*.uplugin")), recursive=True):
            plugin_name = os.path.splitext(os.path.basename(uplugin))[0]
            assert plugin_name not in plugin_dirs
            plugin_dirs[plugin_name] = path_func(os.path.dirname(uplugin))

    return plugin_dirs

def get_filesystem_path_from_content_path(content_path, unreal_project_dir=None, unreal_engine_dir=None, plugin_dirs=None, follow_symlinks=False):

    if follow_symlinks:
        path_func = realpath
    else:
        path_func = lambda path: path

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        assert unreal_project_dir is not None
        filesystem_base_dir = os.path.join(unreal_project_dir, "Content")

    elif content_root == "Engine":
        assert unreal_engine_dir is not None
        filesystem_base_dir = os.path.join(unreal_engine_dir, "Engine", "Content")

    else:
        assert plugin_dirs is not None
        assert content_root in plugin_dirs
        filesystem_base_dir = path_func(os.path.join(plugin_dirs[content_root], "Content"))

    # try to follow Unreal's rules for resolving asset paths

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = path_func(os.path.join(filesystem_base_dir, content_sub_path))
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        elif os.path.islink(filesystem_path) and os.path.exists(realpath(filesystem_path)):
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) in [1, 2]:
                if len(content_file_tokens) == 2:
                    assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(path_func(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], f"{content_file_tokens[0]}.*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return path_func(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            else:
                assert False

#
# Functions for converting filesystem names into names that are valid in content paths, and that stay within Unreal's
# limits for content path lengths.
#

# Unreal's stated maximum path length is 210 chars, but the Unreal Editor adds up to 6 chars when reporting the length
# of a path, so we leave room for them.
def get_max_content_path_length():
    return 210 - 6

# Unreal replaces each of these chars with "_" when it sanitizes a content path (see INVALID_LONGPACKAGE_CHARACTERS in
# Engine/Source/Runtime/Core/Public/UObject/NameTypes.h and UPackageTools::SanitizePackageName(...)).
_unreal_invalid_content_path_chars = "\\:*?\"<>|' ,.&!~\n\r\t@#"

# Applies user_replace_chars_map first, then replaces the chars that Unreal considers invalid in the same way that Unreal
# does, and then truncates the result from the middle if max_length is not None.
def get_content_name_from_filesystem_name(filesystem_name, user_replace_chars_map=None, max_length=None):
    content_name = filesystem_name
    if user_replace_chars_map is not None:
        for k, v in user_replace_chars_map.items():
            content_name = content_name.replace(k, v)
    for c in _unreal_invalid_content_path_chars:
        content_name = content_name.replace(c, "_")
    if max_length is not None:
        content_name = _truncate_from_middle(string=content_name, max_length=max_length)
    return content_name

def _truncate_from_middle(string, max_length):
    truncate_string = "__"
    if len(string) <= max_length:
        return string
    elif max_length <= len(truncate_string):
        return truncate_string[:max_length]
    else:
        return string[:max_length//2-1] + truncate_string + string[-max_length//2+1:]
