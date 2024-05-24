#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--perforce_content_dir")
    parser.add_argument("--scene_id", required=True)
    parser.add_argument("--remove_symlinks", action="store_true")
    args = parser.parse_args()

    # Only check the validity of args.perforce_content_dir if we need to create symlinks to it.
    if not args.remove_symlinks:
        assert args.perforce_content_dir is not None
        assert os.path.exists(args.perforce_content_dir)

    # Replace None with "" because then perforce_content_dir will interact cleanly with os.path.join in all cases.
    if args.perforce_content_dir is None:
        perforce_content_dir = ""
    else:
        perforce_content_dir = args.perforce_content_dir

    unreal_project_content_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Content"))
    assert os.path.exists(unreal_project_content_dir)

    # We do not want to use os.path.realpath(...) for the values in this dictionary, because that will resolve
    # to the directory inside the user's Perforce workspace. Instead, we want this path to refer to the symlinked
    # version inside the user's Unreal project directory.
    perforce_dirs_to_unreal_dirs = {
        os.path.realpath(os.path.join(perforce_content_dir, "Megascans")) : os.path.join(unreal_project_content_dir, "Megascans"),
        os.path.realpath(os.path.join(perforce_content_dir, "MSPresets")) : os.path.join(unreal_project_content_dir, "MSPresets"),
        os.path.realpath(os.path.join(perforce_content_dir, "Scenes", args.scene_id)) : os.path.join(unreal_project_content_dir, "Scenes", args.scene_id)
    }

    for perforce_dir, unreal_project_dir in perforce_dirs_to_unreal_dirs.items():

        # Remove existing symlink
        if spear.path_exists(unreal_project_dir):
            spear.log(f"File or directory or symlink exists, removing: {unreal_project_dir}")
            spear.remove_path(unreal_project_dir)

        # Create new symlinks
        if not args.remove_symlinks:
            assert os.path.exists(perforce_dir)
            spear.log(f"Creating symlink: {unreal_project_dir} -> {perforce_dir}")
            os.symlink(perforce_dir, unreal_project_dir)

    spear.log("Done.")
