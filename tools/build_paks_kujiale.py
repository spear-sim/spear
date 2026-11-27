#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import os
import pandas as pd
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--build-config", required=True)
parser.add_argument("--external-content-dir", required=True)
parser.add_argument("--paks-dir", required=True)
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--version-tag", required=True)
parser.add_argument("--skip-build-common-pak", action="store_true")
parser.add_argument("--skip-build-scene-paks", action="store_true")
parser.add_argument("--scene-ids", nargs="*")
parser.add_argument("--build-dir", default=spear.path.realpath(os.path.dirname(__file__), "BUILD"))
parser.add_argument("--conda-env", default="spear-env")
args = parser.parse_args()

assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]
assert os.path.exists(args.external_content_dir)
assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.build_dir)


def get_assets_in_pak_files(unreal_pak_bin, pak_files):
    assets = []
    for pak_file in pak_files:
        assert os.path.exists(pak_file)
        cmd = [unreal_pak_bin, "-List", spear.tools.normalize_unreal_cmd_path(path=pak_file)]
        ps = spear.tools.popen_unreal_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)
        for line in ps.stdout:
            line_split = line.split('"')
            if len(line_split) == 3 and line_split[0] == "LogPakFile: Display: " and line_split[2].startswith(" offset: "):
                asset = line_split[1]
                assets.append(asset)
        return_code = ps.wait()
        ps.stdout.close()
        assert return_code == 0
    return sorted(list(set(assets)))


if __name__ == "__main__":

    if sys.platform == "win32":

        platform       = "Windows"
        unreal_pak_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe")
        default_pak    = spear.path.realpath(args.build_dir, f"SpearSim-Win64-{args.build_config}", "Windows", "SpearSim", "Content", "Paks", "SpearSim-Windows.pak")

    elif sys.platform == "darwin":

        # TODO: Debug and Test builds only work when the engine is compiled from source, so I don't know the default name of the executable for these build configs
        if args.build_config == "DebugGame":
            executable_app = "SpearSim-Mac-DebugGame.app"
        elif args.build_config == "Development":
            executable_app = "SpearSim.app"
        elif args.build_config == "Shipping":
            executable_app = "SpearSim-Mac-Shipping.app"
        else:
            assert False

        platform       = "Mac"
        unreal_pak_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak")
        default_pak    = spear.path.realpath(args.build_dir, f"SpearSim-Mac-{args.build_config}-Unsigned", "Mac", executable_app, "Contents", "UE", "SpearSim", "Content", "Paks", "SpearSim-Mac.pak")

    elif sys.platform == "linux":

        platform       = "Linux"
        unreal_pak_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak")
        default_pak    = spear.path.realpath(args.build_dir, f"SpearSim-Linux-{args.build_config}", "Linux", "SpearSim", "Content", "Paks", "SpearSim-Linux.pak")

    else:
        assert False

    unreal_project_dir = spear.path.realpath(args.build_dir, "spear", "cpp", "unreal_projects", "SpearSim")

    #
    # create directory for include/exclude files
    #

    build_paks_dir = spear.path.realpath(args.build_dir, "paks")
    spear.log("Creating directory if it does not already exist: ", build_paks_dir)
    os.makedirs(build_paks_dir, exist_ok=True)

    #
    # create symlinks for common dirs
    #

    common_content_dirs = [
        os.path.join("Kujiale", "Materials"),
        os.path.join("Kujiale", "Meshes"),
        os.path.join("Kujiale", "Objects"),
        os.path.join("Kujiale", "Textures"),
        "Megascans",
        "MSPresets"]

    for common_content_dir in common_content_dirs:

        update_action = "create"
        cmd = [
            "python",
            spear.path.realpath(os.path.dirname(__file__), "update_symlink_for_external_content.py"),
            "--external-content-dir", spear.path.realpath(args.external_content_dir, common_content_dir),
            "--unreal-project-dir", unreal_project_dir,
            "--unreal-project-content-dir", common_content_dir,
            f"--{update_action}"]
        spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    #
    # build common pak
    #

    if not args.skip_build_common_pak:

        # We do not want to use spear.path.realpath(...) here, because that will resolve to the directory inside the
        # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
        # Unreal project directory.
        cook_dirs_file = spear.path.realpath(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_cook_dirs.csv")
        cook_dirs = [
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Materials"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Meshes"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Objects"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Textures"),
            os.path.join(unreal_project_dir, "Content", "Megascans"),
            os.path.join(unreal_project_dir, "Content", "MSPresets")]
        df = pd.DataFrame(columns=["cook_dirs"], data={"cook_dirs": cook_dirs})
        df.to_csv(cook_dirs_file, index=False)

        include_assets_file = spear.path.realpath(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_include_assets.csv")
        include_assets = [
            os.path.join("Engine", "Content", "**", "*.*"),
            os.path.join("Engine", "Plugins", "**", "*.*"),
            os.path.join("SpearSim", "Content", "Kujiale", "Materials", "**", "*.*"),
            os.path.join("SpearSim", "Content", "Kujiale", "Meshes", "**", "*.*"),
            os.path.join("SpearSim", "Content", "Kujiale", "Objects", "**", "*.*"),
            os.path.join("SpearSim", "Content", "Kujiale", "Textures", "**", "*.*"),
            os.path.join("SpearSim", "Content", "Megascans", "**", "*.*"),
            os.path.join("SpearSim", "Content", "MSPresets", "**", "*.*")]
        df = pd.DataFrame(columns=["include_assets"], data={"include_assets": include_assets})
        df.to_csv(include_assets_file, index=False)

        exclude_assets_file = spear.path.realpath(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_exclude_assets.csv")
        exclude_pak_files = [default_pak]
        exclude_assets = get_assets_in_pak_files(unreal_pak_bin=unreal_pak_bin, pak_files=exclude_pak_files)
        df = pd.DataFrame(columns=["exclude_assets"], data={"exclude_assets": exclude_assets})
        df.to_csv(exclude_assets_file, index=False)

        pak_file = spear.path.realpath(args.paks_dir, args.version_tag, f"kujiale_common-{args.version_tag}-{platform}.pak")
        cmd = [
            "python",
            spear.path.realpath(os.path.dirname(__file__), "build_pak.py"),
            "--pak-file", pak_file,
            "--cook-dirs-file", cook_dirs_file,
            "--include-assets-file", include_assets_file,
            "--exclude-assets-file", exclude_assets_file,
            "--unreal-engine-dir", args.unreal_engine_dir,
            "--unreal-project-dir", unreal_project_dir]
        spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    #
    # build scene-specific paks
    #

    if not args.skip_build_scene_paks:

        external_content_scenes_dir = spear.path.realpath(args.external_content_dir, "Kujiale", "Scenes")
        ignore_names = [".DS_Store"]
        candidate_scene_ids = [ os.path.basename(x) for x in sorted(os.listdir(external_content_scenes_dir)) if x not in ignore_names ]

        if args.scene_ids is None:
            scene_ids = candidate_scene_ids
        else:
            matched_scene_ids = []
            for candidate_scene_id in candidate_scene_ids:
                for arg_scene_id in args.scene_ids:
                    if fnmatch.fnmatch(candidate_scene_id, arg_scene_id):
                        matched_scene_ids.append(candidate_scene_id)
                        break
            scene_ids = matched_scene_ids

        for scene_id in scene_ids:

            scene_content_dir = os.path.join("Kujiale", "Scenes", scene_id)

            # create symlink
            update_action = "create"
            cmd = [
                "python",
                spear.path.realpath(os.path.dirname(__file__), "update_symlink_for_external_content.py"),
                "--external-content-dir", spear.path.realpath(args.external_content_dir, scene_content_dir),
                "--unreal-project-dir", unreal_project_dir,
                "--unreal-project-content-dir", scene_content_dir,
                f"--{update_action}"]
            spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

            # build pak

            # We do not want to use spear.path.realpath(...) here, because that will resolve to the directory inside the
            # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
            # Unreal project directory.
            cook_dirs_file = spear.path.realpath(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_cook_dirs.csv")
            cook_dirs = [
                os.path.join(unreal_project_dir, "Content", "Kujiale", "Materials"),
                os.path.join(unreal_project_dir, "Content", "Kujiale", "Meshes"),
                os.path.join(unreal_project_dir, "Content", "Kujiale", "Objects"),
                os.path.join(unreal_project_dir, "Content", "Kujiale", "Scenes", scene_id),
                os.path.join(unreal_project_dir, "Content", "Kujiale", "Textures"),
                os.path.join(unreal_project_dir, "Content", "Megascans"),
                os.path.join(unreal_project_dir, "Content", "MSPresets")]
            df = pd.DataFrame(columns=["cook_dirs"], data={"cook_dirs": cook_dirs})
            df.to_csv(cook_dirs_file, index=False)

            cook_maps_file = spear.path.realpath(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_cook_maps.csv")
            cook_maps = [scene_id]
            df = pd.DataFrame(columns=["cook_maps"], data={"cook_maps": cook_maps})
            df.to_csv(cook_maps_file, index=False)

            include_assets_file = spear.path.realpath(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_include_assets.csv")
            include_assets = [
                os.path.join("Engine", "Content", "**", "*.*"),
                os.path.join("Engine", "Plugins", "**", "*.*"),
                os.path.join("SpearSim", "Content", "Kujiale", "Materials", "**", "*.*"),
                os.path.join("SpearSim", "Content", "Kujiale", "Meshes", "**", "*.*"),
                os.path.join("SpearSim", "Content", "Kujiale", "Objects", "**", "*.*"),
                os.path.join("SpearSim", "Content", "Kujiale", "Scenes", scene_id, "**", "*.*"),
                os.path.join("SpearSim", "Content", "Kujiale", "Textures", "**", "*.*"),
                os.path.join("SpearSim", "Content", "Megascans", "**", "*.*"),
                os.path.join("SpearSim", "Content", "MSPresets", "**", "*.*")]
            df = pd.DataFrame(columns=["include_assets"], data={"include_assets": include_assets})
            df.to_csv(include_assets_file, index=False)

            exclude_assets_file = spear.path.realpath(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_exclude_assets.csv")
            exclude_pak_files = [default_pak, spear.path.realpath(args.paks_dir, args.version_tag, f"kujiale_common-{args.version_tag}-{platform}.pak")]
            exclude_assets = get_assets_in_pak_files(unreal_pak_bin=unreal_pak_bin, pak_files=exclude_pak_files)
            df = pd.DataFrame(columns=["exclude_assets"], data={"exclude_assets": exclude_assets})
            df.to_csv(exclude_assets_file, index=False)

            pak_file = spear.path.realpath(args.paks_dir, args.version_tag, f"{scene_id}-{args.version_tag}-{platform}.pak")
            cmd = [
                "python",
                spear.path.realpath(os.path.dirname(__file__), "build_pak.py"),
                "--pak-file", pak_file,
                "--cook-dirs-file", cook_dirs_file,
                "--cook-maps-file", cook_maps_file,
                "--include-assets-file", include_assets_file,
                "--exclude-assets-file", exclude_assets_file,
                "--unreal-engine-dir", args.unreal_engine_dir,
                "--unreal-project-dir", unreal_project_dir]
            spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

            # remove symlink
            update_action = "remove"
            cmd = [
                "python",
                spear.path.realpath(os.path.dirname(__file__), "update_symlink_for_external_content.py"),
                "--external-content-dir", spear.path.realpath(args.external_content_dir, scene_content_dir),
                "--unreal-project-dir", unreal_project_dir,
                "--unreal-project-content-dir", scene_content_dir,
                f"--{update_action}"]
            spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    #
    # remove symlinks for common dirs
    #

    for common_content_dir in common_content_dirs:

        update_action = "remove"
        cmd = [
            "python",
            spear.path.realpath(os.path.dirname(__file__), "update_symlink_for_external_content.py"),
            "--external-content-dir", spear.path.realpath(args.external_content_dir, common_content_dir),
            "--unreal-project-dir", unreal_project_dir,
            "--unreal-project-content-dir", common_content_dir,
            f"--{update_action}"]
        spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    spear.log("Done.")
