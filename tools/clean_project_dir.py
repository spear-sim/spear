#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import spear
import sys


if __name__ == "__main__":

    unreal_project_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
    assert os.path.exists(unreal_project_dir)

    # entire directories to be removed
    dirs = [
        os.path.realpath(os.path.join(unreal_project_dir, "Binaries")),
        os.path.realpath(os.path.join(unreal_project_dir, "DerivedDataCache")),
        os.path.realpath(os.path.join(unreal_project_dir, "Intermediate")),
        os.path.realpath(os.path.join(unreal_project_dir, "Saved"))]

    # individual files to be removed
    files = []

    # plugin files and directories
    plugins_dir = os.path.realpath(os.path.join(unreal_project_dir, "Plugins"))
    is_valid_plugin_dir = lambda p: \
        os.path.isdir(os.path.realpath(os.path.join(plugins_dir, p))) and os.path.exists(os.path.realpath(os.path.join(plugins_dir, p, p + ".uplugin")))
    plugins = [ p for p in sorted(os.listdir(plugins_dir)) if is_valid_plugin_dir(p) ]

    for plugin in plugins:
        plugin_dir = os.path.realpath(os.path.join(plugins_dir, plugin))
        dirs.extend([
            os.path.realpath(os.path.join(plugin_dir, "Binaries")),
            os.path.realpath(os.path.join(plugin_dir, "Intermediate"))])

    # platform-specific files and directories
    if sys.platform == "win32":
        dirs.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build"))])

    elif sys.platform == "darwin":
        dirs.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "FileOpenOrder"))])
        files.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "Resources", "Info.Template.plist")),
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "SpearSim.PackageVersionCounter"))])

    else:
        dirs.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build"))])

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
