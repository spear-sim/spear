#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import pathlib
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--create", action="store_true")
    parser.add_argument("--remove", action="store_true")
    parser.add_argument("--external_content_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--unreal_project_content_dir", required=True)
    parser.add_argument("--skip_create_directories", action="store_true")
    parser.add_argument("--skip_remove_directories", action="store_true")
    args = parser.parse_args()

    assert args.create + args.remove == 1

    external_content_dir = os.path.realpath(args.external_content_dir)

    # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the
    # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
    # Unreal project directory.
    unreal_project_content_root_dir = os.path.join(args.unreal_project_dir, "Content")
    unreal_project_content_dir = os.path.join(unreal_project_content_root_dir, args.unreal_project_content_dir)
    unreal_project_content_base_dir = os.path.split(unreal_project_content_dir)[0]

    # create directories
    if args.create and not args.skip_create_directories:
        spear.log("Creating directory if it does not already exist: ", external_content_dir)
        os.makedirs(external_content_dir, exist_ok=True)

        spear.log("Creating directory if it does not already exist: ", unreal_project_content_base_dir)
        os.makedirs(unreal_project_content_base_dir, exist_ok=True)

    # remove existing symlink
    if spear.path_exists(unreal_project_content_dir):
        spear.log(f"File or directory or symlink exists, removing: {unreal_project_content_dir}")
        spear.remove_path(unreal_project_content_dir)

    # create new symlink
    if args.create:
        spear.log(f"Creating symlink: {unreal_project_content_dir} -> {external_content_dir}")
        os.symlink(external_content_dir, unreal_project_content_dir)

    # If --remove is passed in, then after removing the symlink, we attempt to the remove all of the
    # directories we created when --create was passed in. The heuristic we use here is imperfect, becasue we
    # don't know exactly which directories were created by --create. As a result, we might remove more
    # directories than were created by --create, but we only ever remove empty directories, so we don't think
    # it is especially harmful if this happens occasionally. In other words, we think our heuristic is worth
    # implementing, since it will make it so --create and --remove exactly undo each others' side-effects in
    # nearly all typical use cases. If this behavior is undesired, the user can pass in --skip_create_directories
    # and --skip_remove_directories, and implement their own logic for creating and removing directories in a
    # separate script.

    if args.remove and not args.skip_remove_directories:
        unreal_project_content_root_dir_parts = list(pathlib.Path(unreal_project_content_root_dir).parts)
        unreal_project_content_base_dir_parts = list(pathlib.Path(unreal_project_content_base_dir).parts)
        while unreal_project_content_base_dir_parts != unreal_project_content_root_dir_parts:
            current_part = unreal_project_content_base_dir_parts.pop()
            current_dir = pathlib.Path(*unreal_project_content_base_dir_parts, current_part)
            assert os.path.isdir(current_dir)
            if len(os.listdir(current_dir)) == 0:
                spear.log("Removing empty directory: ", current_dir)
                os.rmdir(current_dir)

        external_content_dir_parts = list(pathlib.Path(unreal_project_content_root_dir).parts)
        while len(external_content_dir_parts) != 0:
            current_part = external_content_dir_parts.pop()
            current_dir = pathlib.Path(*external_content_dir_parts, current_part)
            assert os.path.isdir(current_dir)
            if len(os.listdir(current_dir)) == 0:
                spear.log("Removing empty directory: ", current_dir)
                os.rmdir(current_dir)

    spear.log("Done.")
