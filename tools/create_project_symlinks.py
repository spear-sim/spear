#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import spear


plugins = ["SpComponents", "SpCore", "SpModuleRules", "SpServices", "UrdfRobot", "Vehicle"]


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--unreal_plugins_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_plugins")))
    parser.add_argument("--third_party_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "third_party")))
    args = parser.parse_args()

    assert os.path.exists(args.unreal_project_dir)
    assert os.path.exists(args.unreal_plugins_dir)
    assert os.path.exists(args.third_party_dir)

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    unreal_plugins_dir = os.path.realpath(args.unreal_plugins_dir)
    third_party_dir    = os.path.realpath(args.third_party_dir)

    # verify that our project is present
    _, project = os.path.split(unreal_project_dir)
    uproject = os.path.realpath(os.path.join(unreal_project_dir, project + ".uproject"))
    assert os.path.exists(uproject)
    spear.log(f"Found uproject: {uproject}")

    # verify that each of our plugins is present
    for plugin in plugins:
        uplugin = os.path.realpath(os.path.join(unreal_plugins_dir, plugin, plugin + ".uplugin"))
        assert os.path.exists(uplugin)
        spear.log(f"Found uplugin: {uplugin}")

    # create a ThirdParty symlink in the project dir
    symlink_third_party_dir = os.path.join(unreal_project_dir, "ThirdParty") # don't want os.path.realpath here
    if spear.path_exists(symlink_third_party_dir):
        spear.log(f"File or directory or symlink exists, removing: {symlink_third_party_dir}")
        spear.remove_path(symlink_third_party_dir)
    spear.log(f"Creating symlink: {symlink_third_party_dir} -> {third_party_dir}")
    os.symlink(third_party_dir, symlink_third_party_dir)

    # create a Plugins dir in the project dir
    project_plugins_dir = os.path.realpath(os.path.join(unreal_project_dir, "Plugins"))
    os.makedirs(project_plugins_dir, exist_ok=True)

    # for each plugin
    for plugin in plugins:

        spear.log(f"    Creating symlinks for plugin: {plugin}")

        # create a symlink in the Plugins dir
        plugin_dir = os.path.realpath(os.path.join(unreal_plugins_dir, plugin))
        symlink_plugin_dir = os.path.join(project_plugins_dir, plugin) # don't want os.path.realpath here
        if spear.path_exists(symlink_plugin_dir):
            spear.log(f"        File or directory or symlink exists, removing: {symlink_plugin_dir}")
            spear.remove_path(symlink_plugin_dir)
        spear.log(f"        Creating symlink: {symlink_plugin_dir} -> {plugin_dir}")
        os.symlink(plugin_dir, symlink_plugin_dir)

        # create a ThirdParty symlink in the plugin dir
        symlink_third_party_dir = os.path.join(unreal_plugins_dir, plugin, "ThirdParty") # don't want os.path.realpath here
        if spear.path_exists(symlink_third_party_dir):
            spear.log(f"        File or directory or symlink exists, removing: {symlink_third_party_dir}")
            spear.remove_path(symlink_third_party_dir)
        spear.log(f"        Creating symlink: {symlink_third_party_dir} -> {third_party_dir}")
        os.symlink(third_party_dir, symlink_third_party_dir)

    spear.log("Done.")
