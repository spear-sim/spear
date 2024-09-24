#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_project_content_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--external_content_dir")
    parser.add_argument("--create", action="store_true")
    parser.add_argument("--remove", action="store_true")
    args = parser.parse_args()


    assert args.create + args.remove == 1

    # Only check the validity of args.external_content_dir if we need to create symlinks to it.
    if args.create:
        assert args.external_content_dir is not None
        external_content_dir = os.path.realpath(args.external_content_dir)
        os.makedirs(external_content_dir, exist_ok=True)

    # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the
    # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
    # Unreal project directory.
    unreal_project_content_dir = os.path.join(args.unreal_project_dir, "Content", args.unreal_project_content_dir)
    unreal_project_content_base_dir = os.path.split(unreal_project_content_dir)[0]

    assert os.path.exists(unreal_project_content_base_dir)

    # Remove existing symlink
    if spear.path_exists(unreal_project_content_dir):
        spear.log(f"File or directory or symlink exists, removing: {unreal_project_content_dir}")
        spear.remove_path(unreal_project_content_dir)

    # Create new symlink
    if args.create:
        spear.log(f"Creating symlink: {unreal_project_content_dir} -> {external_content_dir}")
        os.symlink(external_content_dir, unreal_project_content_dir)

    spear.log("Done.")
