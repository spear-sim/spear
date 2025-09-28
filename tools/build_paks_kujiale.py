#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import ntpath
import os
import pandas as pd
import posixpath
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--build_config", required=True)
parser.add_argument("--external_content_dir", required=True)
parser.add_argument("--paks_dir", required=True)
parser.add_argument("--unreal_engine_dir", required=True)
parser.add_argument("--version_tag", required=True)
parser.add_argument("--conda_script")
parser.add_argument("--skip_build_common_pak", action="store_true")
parser.add_argument("--skip_build_scene_paks", action="store_true")
parser.add_argument("--scene_ids", nargs="*")
parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
parser.add_argument("--conda_env", default="spear-env")
args = parser.parse_args()

assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]
assert os.path.exists(args.external_content_dir)
assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.build_dir)


if __name__ == "__main__":

    if sys.platform == "win32":

        platform       = "Windows"
        unreal_pak_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe"))
        default_pak    = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Win64-{args.build_config}", "Windows", "SpearSim", "Content", "Paks", "SpearSim-Windows.pak"))
        cmd_prefix     = f"conda activate {args.conda_env} & "

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
        unreal_pak_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak"))
        default_pak    = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Mac-{args.build_config}-Unsigned", "Mac", executable_app, "Contents", "UE", "SpearSim", "Content", "Paks", "SpearSim-Mac.pak"))

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),  # anaconda shell install
                os.path.join(os.sep, "opt", "anaconda3", "etc", "profile.d", "conda.sh"),            # anaconda graphical install
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh")), # miniconda shell install
                os.path.join(os.sep, "opt", "miniconda3", "etc", "profile.d", "conda.sh")]           # miniconda graphical install

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    elif sys.platform == "linux":

        platform       = "Linux"
        unreal_pak_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak"))
        default_pak    = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Linux-{args.build_config}", "Linux", "SpearSim", "Content", "Paks", "SpearSim-Linux.pak"))

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh"))]

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    else:
        assert False

    unreal_project_dir = os.path.realpath(os.path.join(args.build_dir, "spear", "cpp", "unreal_projects", "SpearSim"))

    #
    # create directory for include/exclude files
    #

    build_paks_dir = os.path.realpath(os.path.join(args.build_dir, "paks"))
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
        cmd = \
            cmd_prefix + \
            "python " + \
            f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "update_symlinks_for_external_content.py"))}" ' + \
            f'--external_content_dir "{os.path.realpath(os.path.join(args.external_content_dir, common_content_dir))}" ' + \
            f'--unreal_project_dir "{unreal_project_dir}" ' + \
            f'--unreal_project_content_dir "{common_content_dir}" ' + \
            f"--{update_action}"
        spear.log("Executing: ", cmd)
        subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

    #
    # build common pak
    #

    if not args.skip_build_common_pak:

        # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the
        # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
        # Unreal project directory.
        cook_dirs_file = os.path.realpath(os.path.join(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_cook_dirs.csv"))
        cook_dirs = [
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Materials"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Meshes"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Objects"),
            os.path.join(unreal_project_dir, "Content", "Kujiale", "Textures"),
            os.path.join(unreal_project_dir, "Content", "Megascans"),
            os.path.join(unreal_project_dir, "Content", "MSPresets")]
        df = pd.DataFrame(columns=["cook_dirs"], data={"cook_dirs": cook_dirs})
        df.to_csv(cook_dirs_file, index=False)

        include_assets_file = os.path.realpath(os.path.join(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_include_assets.csv"))
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

        exclude_assets_file = os.path.realpath(os.path.join(build_paks_dir, f"kujiale_common-{args.version_tag}-{platform}_exclude_assets.csv"))
        exclude_pak_files = [default_pak]
        exclude_assets = []
        for exclude_pak_file in exclude_pak_files:
            cmd = [unreal_pak_bin, "-List", exclude_pak_file]
            spear.log("Executing: ", ' '.join(cmd))
            ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
            for line in ps.stdout:
                line_split = line.split('"')
                if len(line_split) == 3 and line_split[0] == "LogPakFile: Display: " and line_split[2].startswith(" offset: "):
                    asset = line_split[1]
                    exclude_assets.append(asset)
            ps.wait()
            ps.stdout.close()
        exclude_assets = sorted(list(set(exclude_assets)))
        df = pd.DataFrame(columns=["exclude_assets"], data={"exclude_assets": exclude_assets})
        df.to_csv(exclude_assets_file, index=False)

        pak_file = os.path.realpath(os.path.join(args.paks_dir, args.version_tag, f"kujiale_common-{args.version_tag}-{platform}.pak"))
        cmd = \
            cmd_prefix + \
            "python " + \
            f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "build_pak.py"))}" ' + \
            f'--pak_file "{pak_file}" ' + \
            f'--cook_dirs_file "{cook_dirs_file}" ' + \
            f'--include_assets_file "{include_assets_file}" ' + \
            f'--exclude_assets_file "{exclude_assets_file}" ' + \
            f'--unreal_engine_dir "{args.unreal_engine_dir}" ' + \
            f'--unreal_project_dir "{unreal_project_dir}"'
        spear.log("Executing: ", cmd)
        subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

    #
    # build scene-specific paks
    #

    if not args.skip_build_scene_paks:

        external_content_scenes_dir = os.path.realpath(os.path.join(args.external_content_dir, "Kujiale", "Scenes"))
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
            cmd = \
                cmd_prefix + \
                "python " + \
                f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "update_symlinks_for_external_content.py"))}" ' + \
                f'--external_content_dir "{os.path.realpath(os.path.join(args.external_content_dir, scene_content_dir))}" ' + \
                f'--unreal_project_dir "{unreal_project_dir}" ' + \
                f'--unreal_project_content_dir "{scene_content_dir}" ' + \
                f"--{update_action}"
            spear.log("Executing: ", cmd)
            subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

            # build pak

            # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the
            # user's external directory. Instead, we want this path to refer to the symlinked path inside the user's
            # Unreal project directory.
            cook_dirs_file = os.path.realpath(os.path.join(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_cook_dirs.csv"))
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

            cook_maps_file = os.path.realpath(os.path.join(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_cook_maps.csv"))
            cook_maps = [scene_id]
            df = pd.DataFrame(columns=["cook_maps"], data={"cook_maps": cook_maps})
            df.to_csv(cook_maps_file, index=False)

            include_assets_file = os.path.realpath(os.path.join(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_include_assets.csv"))
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

            exclude_assets_file = os.path.realpath(os.path.join(build_paks_dir, f"{scene_id}-{args.version_tag}-{platform}_exclude_assets.csv"))
            exclude_pak_files = [default_pak, os.path.realpath(os.path.join(args.paks_dir, args.version_tag, f"kujiale_common-{args.version_tag}-{platform}.pak"))]
            exclude_assets = []
            for exclude_pak_file in exclude_pak_files:
                assert os.path.exists(exclude_pak_file)
                cmd = [unreal_pak_bin, "-List", exclude_pak_file]
                spear.log("Executing: ", ' '.join(cmd))
                ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
                for line in ps.stdout:
                    line_split = line.split('"')
                    if len(line_split) == 3 and line_split[0] == "LogPakFile: Display: " and line_split[2].startswith(" offset: "):
                        asset = line_split[1]
                        exclude_assets.append(asset)
                ps.wait()
                ps.stdout.close()
            exclude_assets = sorted(list(set(exclude_assets)))
            df = pd.DataFrame(columns=["exclude_assets"], data={"exclude_assets": exclude_assets})
            df.to_csv(exclude_assets_file, index=False)

            pak_file = os.path.realpath(os.path.join(args.paks_dir, args.version_tag, f"{scene_id}-{args.version_tag}-{platform}.pak"))
            cmd = \
                cmd_prefix + \
                "python " + \
                f'{os.path.realpath(os.path.join(os.path.dirname(__file__), "build_pak.py"))} ' + \
                f'--pak_file "{pak_file}" ' + \
                f'--cook_dirs_file "{cook_dirs_file}" ' + \
                f'--cook_maps_file "{cook_maps_file}" ' + \
                f'--include_assets_file "{include_assets_file}" ' + \
                f'--exclude_assets_file "{exclude_assets_file}" ' + \
                f'--unreal_engine_dir "{args.unreal_engine_dir}" ' + \
                f'--unreal_project_dir "{unreal_project_dir}"'
            spear.log("Executing: ", cmd)
            subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

            # remove symlink
            update_action = "remove"
            cmd = \
                cmd_prefix + \
                "python " + \
                f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "update_symlinks_for_external_content.py"))}" ' + \
                f'--external_content_dir "{os.path.realpath(os.path.join(args.external_content_dir, scene_content_dir))}" ' + \
                f'--unreal_project_dir "{unreal_project_dir}" ' + \
                f'--unreal_project_content_dir "{scene_content_dir}" ' + \
                f"--{update_action}"
            spear.log("Executing: ", cmd)
            subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

    #
    # remove symlinks for common dirs
    #

    for common_content_dir in common_content_dirs:

        update_action = "remove"
        cmd = \
            cmd_prefix + \
            "python " + \
            f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "update_symlinks_for_external_content.py"))}" ' + \
            f'--external_content_dir "{os.path.realpath(os.path.join(args.external_content_dir, common_content_dir))}" ' + \
            f'--unreal_project_dir "{unreal_project_dir}" ' + \
            f'--unreal_project_content_dir "{common_content_dir}" ' + \
            f"--{update_action}"
        spear.log("Executing: ", cmd)
        subprocess.run(cmd, shell=True, check=True) # we need shell=True because we want to run in a specific anaconda env

    spear.log("Done.")
