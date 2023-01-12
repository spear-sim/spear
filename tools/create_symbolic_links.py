#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_project_dir")
    args = parser.parse_args()

    tools_dir           = os.path.dirname(os.path.realpath(__file__))
    unreal_projects_dir = os.path.realpath(os.path.join(tools_dir, "..", "cpp", "unreal_projects"))
    unreal_plugins_dir  = os.path.realpath(os.path.join(tools_dir, "..", "cpp", "unreal_plugins"))
    third_party_dir     = os.path.realpath(os.path.join(tools_dir, "..", "third_party"))

    if args.unreal_project_dir:
        unreal_project_dirs = [ os.path.realpath(args.unreal_project_dir) ]
    else:
        unreal_project_dirs = [ os.path.join(unreal_projects_dir, project) for project in os.listdir(unreal_projects_dir) ]

    unreal_plugins = os.listdir(unreal_plugins_dir)

    # for each plugin...
    for plugin in unreal_plugins:

        # ...if the plugin is a valid plugin (i.e., plugin dir has a uplugin file)
        uplugin = os.path.join(unreal_plugins_dir, plugin, plugin + ".uplugin")
        if os.path.exists(uplugin):

            print(f"[SPEAR | create_sybmolic_links.py] Found uplugin: {uplugin}")

            # create a symlink to third_party
            symlink_third_party_dir = os.path.join(unreal_plugins_dir, plugin, "ThirdParty")
            if spear.path_exists(symlink_third_party_dir):
                print(f"[SPEAR | create_sybmolic_links.py]     File or directory or symlink exists, removing: {symlink_third_party_dir}")
                spear.remove_path(symlink_third_party_dir)
            print(f"[SPEAR | create_sybmolic_links.py]     Creating symlink: {symlink_third_party_dir} -> {third_party_dir}")
            os.symlink(third_party_dir, symlink_third_party_dir)
            print()

    # for each project...
    for unreal_project_dir in unreal_project_dirs:

        assert os.path.exists(unreal_project_dir)

        # ...if the project is a valid project (i.e., project dir has a uproject file)
        _, project = os.path.split(unreal_project_dir)
        uproject = os.path.join(unreal_project_dir, project + ".uproject")
        if os.path.exists(uproject):

            print(f"[SPEAR | create_sybmolic_links.py] Found uproject: {uproject}")

            # create a symlink to third_party
            symlink_third_party_dir = os.path.join(unreal_projects_dir, project, "ThirdParty")
            if spear.path_exists(symlink_third_party_dir):
                print(f"[SPEAR | create_sybmolic_links.py]     File or directory or symlink exists, removing: {symlink_third_party_dir}")
                spear.remove_path(symlink_third_party_dir)
            print(f"[SPEAR | create_sybmolic_links.py]     Creating symlink: {symlink_third_party_dir} -> {third_party_dir}")
            os.symlink(third_party_dir, symlink_third_party_dir)

            # get list of plugins from the uproject file
            with open(os.path.join(unreal_projects_dir, project, project + ".uproject")) as f:
                project_plugins = [ p["Name"] for p in json.load(f)["Plugins"] ]
            print(f"[SPEAR | create_sybmolic_links.py]     Plugin dependencies: {project_plugins}")

            # create a Plugins dir in the project dir
            project_plugins_dir = os.path.join(unreal_projects_dir, project, "Plugins")
            os.makedirs(project_plugins_dir, exist_ok=True)

            # create symlink for each plugin listed in the project, if the plugin is in our unreal_plugins dir
            for project_plugin in project_plugins:
                if project_plugin in unreal_plugins:
                    plugin_dir = os.path.abspath(os.path.join(unreal_plugins_dir, project_plugin))
                    symlink_plugin_dir = os.path.abspath(os.path.join(project_plugins_dir, project_plugin))
                    if spear.path_exists(symlink_plugin_dir):
                        print(f"[SPEAR | create_sybmolic_links.py]         File or directory or symlink exists, removing: {symlink_plugin_dir}")
                        spear.remove_path(symlink_plugin_dir)
                    print(f"[SPEAR | create_sybmolic_links.py]         Creating symlink: {symlink_plugin_dir} -> {plugin_dir}")
                    os.symlink(plugin_dir, symlink_plugin_dir)
                else:
                    print(f"[SPEAR | create_sybmolic_links.py]         {project} depends on {project_plugin}, but this plugin is not in {unreal_plugins_dir}, so we do not create a symlink...")

            print()

    print("[SPEAR | create_sybmolic_links.py] Done.")
 