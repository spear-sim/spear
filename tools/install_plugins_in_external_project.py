#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import json
import os
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--external-project-dir")
args = parser.parse_args()


if __name__ == "__main__":

    external_project_dir = os.path.realpath(args.external_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(external_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    uproject_dir = os.path.dirname(uproject)

    uproject_dict = {}
    with open(uproject) as f:
        uproject_dict = json.load(f)

    save = False
    target_plugins_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_plugins")).replace("\\\\", "/").replace("\\", "/")

    if "AdditionalPluginDirectories" in uproject_dict:
        found = False
        for additional_plugins_dir in uproject_dict["AdditionalPluginDirectories"]:
            candidate_plugins_dir = additional_plugins_dir.replace("/", os.path.sep)
            if os.path.isabs(candidate_plugins_dir):
                candidate_plugins_dir = candidate_plugins_dir.replace("\\\\", "/").replace("\\", "/")
            else:
                candidate_plugins_dir = os.path.realpath(os.path.join(uproject_dir, candidate_plugins_dir)).replace("\\\\", "/").replace("\\", "/")
            if candidate_plugins_dir == target_plugins_dir:
                spear.log("AdditionalPluginDirectories already has a matching entry: ", additional_plugins_dir)
                found = True
                break
        if not found:
            spear.log("Adding to AdditionalPluginDirectories: ", target_plugins_dir)
            uproject_dict["AdditionalPluginDirectories"].append(target_plugins_dir)
            save = True
    else:
        spear.log("Creating AdditionalPluginDirectories and adding: ", target_plugins_dir)
        uproject_dict["AdditionalPluginDirectories"] = [target_plugins_dir]
        save = True

    if save:
        spear.log("Saving uproject: ", uproject)
        with open(uproject, "w") as f:
            json.dump(uproject_dict, f, indent=4, sort_keys=False)
            f.write("\n")

    spear.log("Done.")
