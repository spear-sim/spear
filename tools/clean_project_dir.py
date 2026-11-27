#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import spear
import sys


if __name__ == "__main__":

    project_dir = spear.path.realpath(os.path.dirname(__file__), "..")
    unreal_project_dir = spear.path.realpath(project_dir, "cpp", "unreal_projects", "SpearSim")
    unreal_plugins_dir = spear.path.realpath(project_dir, "cpp", "unreal_plugins")
    assert os.path.exists(project_dir)
    assert os.path.exists(unreal_project_dir)
    assert os.path.exists(unreal_plugins_dir)

    # entire directories to be removed
    dirs = [
        spear.path.realpath(project_dir, "python_ext", "BUILD"),
        spear.path.realpath(project_dir, "tools", "BUILD"),
        spear.path.realpath(unreal_project_dir, "Binaries"),
        spear.path.realpath(unreal_project_dir, "DerivedDataCache"),
        spear.path.realpath(unreal_project_dir, "Intermediate"),
        spear.path.realpath(unreal_project_dir, "Saved"),
        spear.path.realpath(unreal_project_dir, "Standalone-Debug"),
        spear.path.realpath(unreal_project_dir, "Standalone-DebugGame"),
        spear.path.realpath(unreal_project_dir, "Standalone-Development"),
        spear.path.realpath(unreal_project_dir, "Standalone-Shipping"),
        spear.path.realpath(unreal_project_dir, "Standalone-Test")]

    # individual files to be removed
    files = []

    # plugin files and directories
    is_valid_plugin_dir = lambda p: \
        os.path.isdir(spear.path.realpath(unreal_plugins_dir, p)) and os.path.exists(spear.path.realpath(unreal_plugins_dir, p, f"{p}.uplugin"))
    plugins = [ p for p in sorted(os.listdir(unreal_plugins_dir)) if is_valid_plugin_dir(p) ]

    for plugin in plugins:
        plugin_dir = spear.path.realpath(unreal_plugins_dir, plugin)
        dirs.extend([
            spear.path.realpath(plugin_dir, "Binaries"),
            spear.path.realpath(plugin_dir, "Intermediate")])

    # platform-specific files and directories, we remove generated files from the Build directory selectively,
    # because Build/Mac/Resources/entitlements.plist is checked in
    if sys.platform == "win32":
        dirs.extend([
            spear.path.realpath(unreal_project_dir, "Build", "Windows")])

    elif sys.platform == "darwin":
        dirs.extend([
            spear.path.realpath(unreal_project_dir, "Build", "Mac", "FileOpenOrder")])
        files.extend([
            spear.path.realpath(unreal_project_dir, "Build", "Mac", "Resources", "Info.Template.plist"),
            spear.path.realpath(unreal_project_dir, "Build", "Mac", "SpearSim.PackageVersionCounter")])

    elif sys.platform == "linux":
        dirs.extend([
            spear.path.realpath(unreal_project_dir, "Build", "Linux")])

    else:
        assert False

    # remove dirs
    for d in dirs:
        if os.path.exists(d):
            spear.log("Removing: ", d)
            shutil.rmtree(d, ignore_errors=True)

    # remove files
    for f in files:
        if os.path.exists(f):
            spear.log("Removing: ", f)
            os.remove(f)

    spear.log("Done.")
