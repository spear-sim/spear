#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--perforce_content_dir")
    parser.add_argument("--scene_id", required=True)
    parser.add_argument("--remove_symlinks", action="store_true")
    args = parser.parse_args()
    
    if not args.remove_symlinks:
        assert args.perforce_content_dir is not None
        assert os.path.exists(args.perforce_content_dir)

        perforce_content_shared_dir = os.path.realpath(os.path.join(args.perforce_content_dir, "Shared"))
        perforce_content_scene_dir = os.path.realpath(os.path.join(args.perforce_content_dir, "Scenes", args.scene_id))

        assert os.path.exists(perforce_content_shared_dir)
        assert os.path.exists(perforce_content_scene_dir)

    unreal_project_content_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Content"))

    # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
    # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
    unreal_project_content_shared_dir = os.path.join(unreal_project_content_dir, "Shared")
    unreal_project_content_scene_dir = os.path.join(unreal_project_content_dir, "Scenes", args.scene_id)

    # Remove existing symlink for the Shared directory
    if spear.path_exists(unreal_project_content_shared_dir):
        print(f"[SPEAR | prepare_scene_for_editing.py] File or directory or symlink exists, removing: {unreal_project_content_shared_dir}")
        spear.remove_path(unreal_project_content_shared_dir)

    # Remove existing symlink for the scene directory
    if spear.path_exists(unreal_project_content_scene_dir):
        print(f"[SPEAR | prepare_scene_for_editing.py] File or directory or symlink exists, removing: {unreal_project_content_scene_dir}")
        spear.remove_path(unreal_project_content_scene_dir)

    # Create new symlinks
    if not args.remove_symlinks:
        print(f"[SPEAR | prepare_scene_for_editing.py] Creating symlink: {unreal_project_content_shared_dir} -> {perforce_content_shared_dir}")
        os.symlink(perforce_content_shared_dir, unreal_project_content_shared_dir)

        print(f"[SPEAR | prepare_scene_for_editing.py] Creating symlink: {unreal_project_content_scene_dir} -> {perforce_content_scene_dir}")
        os.symlink(perforce_content_scene_dir, unreal_project_content_scene_dir)

    print("[SPEAR | prepare_scene_for_editing.py] Done.")
