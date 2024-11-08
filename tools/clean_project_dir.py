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
    plugins = os.listdir(plugins_dir)
    for plugin in plugins:
        plugin_dir = os.path.realpath(os.path.join(plugins_dir, plugin))
        dirs.extend([
            os.path.realpath(os.path.join(plugin_dir, "Binaries")),
            os.path.realpath(os.path.join(plugin_dir, "Intermediate"))])

    # platform-specific files and directories
    if sys.platform == "win32":
        dirs.extend([
            os.path.realpath(os.path.join(unreal_project_dir, ".vs")),
            os.path.realpath(os.path.join(unreal_project_dir, "Build"))])
        files.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.sln"))])

    elif sys.platform == "darwin":
        dirs.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "FileOpenOrder")),
            os.path.realpath(os.path.join(unreal_project_dir, "SpearSim (Mac).xcworkspace"))])
        files.extend([
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "Resources", "Info.Template.plist")),
            os.path.realpath(os.path.join(unreal_project_dir, "Build", "Mac", "SpearSim.PackageVersionCounter")),
            os.path.realpath(os.path.join(unreal_project_dir, "SpearSim (Mac).xcworkspace")),
            os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.sln"))])

    else:
        assert False # TODO: update for Linux

    # remove dirs
    for dir in dirs:
        if os.path.exists(dir):
            spear.log("Removing: ", dir)
            shutil.rmtree(dir, ignore_errors=True)

    # remove files
    for file in files:
        if os.path.exists(file):
            spear.log("Removing: ", file)
            os.remove(file)

    spear.log("Done.")
